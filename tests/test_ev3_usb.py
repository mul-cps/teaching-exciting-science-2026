"""Wire-level tests; no USB device or motor is accessed."""
import json
import struct
from pathlib import Path

import pytest

from workshop.station.ev3_usb import OriginalEV3, DirectUSB, EV3Error
from workshop.station.discovery import discover_ev3, select_identity


def config():
    c = json.loads(Path('workshop/station/resources/station.example.json').read_text())
    c['ev3']['usb']['serial'] = '001122334455'
    c['ev3']['feeder']['home_travel_deg'] = 180
    c['timeouts']['operation_s'] = 0.15
    c['timeouts']['stop_s'] = 0.05
    return c


class Wire:
    def __init__(self, payload=b'', bad=None):
        self.payload, self.bad = payload, bad
        self.writes, self.timeouts = [], []
        self.identity = {'vendor_id': 0x694, 'product_id': 5, 'serial': '001122334455'}
        self.partial = False

    def write(self, endpoint, packet, timeout):
        assert endpoint == 1
        self.timeouts.append(timeout)
        self.writes.append(bytes(packet))
        return len(packet) - int(self.partial)

    def read(self, endpoint, size, timeout):
        assert endpoint == 0x81
        self.timeouts.append(timeout)
        if isinstance(self.bad, BaseException):
            raise self.bad
        if self.bad is not None:
            return self.bad
        counter = self.writes[-1][2:4]
        return struct.pack('<H', 3 + len(self.payload)) + counter + b'\x02' + self.payload

    def close(self):
        pass


def test_framing_is_official_little_endian_and_positive_timeout():
    wire = Wire(b'\x01')
    assert DirectUSB(wire).command(b'\xa9\x00\x01\x60', 1, 0.05) == b'\x01'
    assert wire.writes == [b'\x09\x00\x01\x00\x00\x01\x00\xa9\x00\x01\x60']
    assert all(0 < t <= 50 for t in wire.timeouts)


@pytest.mark.parametrize('bad', [b'', b'\0', b'\x03\0\x01\0', b'\x09\0\x01\0\x02', b'\x03\0\x01\0\x04', b'\x03\0\x01\0\x03', b'\x04\0\x01\0\x02\0', EOFError('lost'), TimeoutError('usb')])
def test_bad_replies_are_rejected_without_write_retry(bad):
    wire = Wire(bad=bad)
    with pytest.raises((EV3Error, EOFError, TimeoutError)):
        DirectUSB(wire).command(b'\xa3\0\x09\x01', 0, 0.01)
    assert len(wire.writes) == 1


def test_partial_write_is_fault_not_retry():
    wire = Wire()
    wire.partial = True
    with pytest.raises(EV3Error, match='partial'):
        DirectUSB(wire).command(b'\xa3\0\x09\x01', 0, 0.05)
    assert len(wire.writes) == 1


def test_unrelated_counter_has_total_deadline():
    wire = Wire(bad=b'\x03\0\xff\xff\x02')
    with pytest.raises(TimeoutError):
        DirectUSB(wire).command(b'\xa3\0\x09\x01', 0, 0.005)
    assert len(wire.writes) == 1
    assert all(t > 0 for t in wire.timeouts)


def test_lock_has_deadline_and_releases_after_failure():
    wire = Wire(bad=b'')
    protocol = DirectUSB(wire)
    protocol.lock.acquire()
    try:
        with pytest.raises(TimeoutError):
            protocol.command(b'', 0, 0.005)
    finally:
        protocol.lock.release()
    with pytest.raises(EV3Error):
        protocol.command(b'', 0, 0.01)
    wire.bad = None
    assert protocol.command(b'', 0, 0.01) == b''


@pytest.mark.parametrize('seconds', [0, -1, float('inf'), float('nan')])
def test_nonfinite_or_zero_deadlines_rejected(seconds):
    with pytest.raises(ValueError):
        DirectUSB(Wire()).command(b'', 0, seconds)


def test_discovery_is_readonly_sysfs_and_serial_by_id(tmp_path):
    usb = tmp_path / 'usb'
    dev = usb / '1-1'
    dev.mkdir(parents=True)
    for name, value in {'idVendor':'0694', 'idProduct':'0005', 'serial':'001122334455', 'busnum':'1', 'devnum':'8', 'product':'EV3'}.items():
        (dev / name).write_text(value)
    serial = tmp_path / 'by-id'
    serial.mkdir()
    (serial / 'usb-arm').symlink_to('/dev/ttyACM0')
    found = discover_ev3(usb, serial)
    assert found[0]['serial'] == '001122334455'
    assert found[0]['bus'] == 1 and found[0]['address'] == 8
    assert (dev / 'serial').read_text() == '001122334455'
    assert select_identity(found, config()['ev3']['usb']) == found[0]


@pytest.mark.parametrize('candidates', [[], [{'vendor_id':0x694, 'product_id':5, 'serial':'other'}], [{'vendor_id':0x694, 'product_id':5, 'serial':'001122334455'}] * 2])
def test_missing_wrong_or_ambiguous_identity_rejected(candidates):
    with pytest.raises(ValueError):
        select_identity(candidates, config()['ev3']['usb'])


class Brick(Wire):
    """Small independent wire emulator for the documented opcode subset."""
    def __init__(self):
        super().__init__()
        self.types = {0:16, 2:29, 16:8, 19:7}
        self.encoders = {16:0., 19:0.}
        self.touch = 0.
        self.color = 2.
        self.motion = []
        self.stop_masks = []
        self.stalled = True

    def read(self, endpoint, size, timeout):
        ops = self.writes[-1][7:]
        if ops[:2] == b'\x99\x05':
            self.payload = bytes([self.types[ops[3]], 0])
        elif ops[:2] == b'\x99\x1d':
            port = ops[3]
            val = self.encoders[port] if port >= 16 else self.touch if port == 0 else self.color
            self.payload = struct.pack('<f', val)
        elif ops[0] == 0x81:
            self.payload = b'V1.09H\0' + b'\0' * 25
        elif ops[0] == 0xa9:
            self.payload = b'\0'
        else:
            self.payload = b''
        return super().read(endpoint, size, timeout)

    def write(self, endpoint, packet, timeout):
        result = super().write(endpoint, packet, timeout)
        ops = packet[7:]
        if ops[0] == 0xaf:
            self.motion.append(ops)
            if not self.stalled:
                # Signed LC1 speed; finite time, no encoder target command.
                direction = struct.unpack('b', ops[4:5])[0]
                port = 16 if ops[2] == 1 else 19
                self.encoders[port] += 5 if direction > 0 else -5
        if ops[0] == 0xa3:
            self.stop_masks.append(ops[2])
        return result


def ready():
    brick = Brick()
    ev3 = OriginalEV3(config(), brick)
    ev3.connect()
    ev3.preflight()
    return ev3, brick


def test_constructor_and_preflight_are_torque_safe():
    ev3, brick = ready()
    assert not brick.motion and not brick.stop_masks
    assert ev3.identity['firmware_version'] == 'V1.09H'
    assert ev3.read_color() == 'blue'


def test_wrong_port_type_prevents_motion():
    brick = Brick()
    brick.types[16] = 7
    ev3 = OriginalEV3(config(), brick)
    ev3.connect()
    with pytest.raises(EV3Error, match='type'):
        ev3.preflight()
    assert not brick.motion


def test_unhomed_feed_rejected():
    ev3, brick = ready()
    with pytest.raises(EV3Error, match='home'):
        ev3.feed()
    assert not brick.motion


def test_stuck_touch_stops_both_motors_without_zeroing():
    ev3, brick = ready()
    brick.touch = 1.
    brick.stalled = False
    with pytest.raises((EV3Error, TimeoutError)):
        ev3.home()
    assert 9 in brick.stop_masks
    assert not ev3.homed
    assert not any(p[7] == 0xb2 for p in brick.writes)
    assert brick.motion
    for ops in brick.motion:
        assert ops[5:7] == b'\x83\0'  # LC4 ramp-up = 0
        duration = struct.unpack('<i', ops[11:15])[0]
        assert 0 < duration <= 100


def test_tracking_stall_and_fault_latch_prevent_feed_replay():
    ev3, brick = ready()
    ev3.homed = True
    with pytest.raises(TimeoutError):
        ev3.feed()
    attempts = len(brick.motion)
    assert attempts and 9 in brick.stop_masks
    with pytest.raises(EV3Error, match='fault'):
        ev3.feed()
    assert len(brick.motion) == attempts


def test_disconnected_motion_attempt_stops_independently():
    ev3, brick = ready()
    ev3.homed = True
    brick.bad = EOFError('disconnected')
    with pytest.raises(EOFError):
        ev3.feed()
    assert 9 in brick.stop_masks
    writes = len(brick.motion)
    with pytest.raises(EV3Error):
        ev3.feed()
    assert len(brick.motion) == writes


def test_stop_allowed_after_fault_and_wait_stopped_checks_busy():
    ev3, brick = ready()
    ev3.faulted = True
    ev3.stop()
    ev3.wait_stopped()
    assert brick.stop_masks == [9]


def test_discover_arm_keeps_exact_usb_identity_and_stable_paths(tmp_path):
    from workshop.station.discovery import discover_arm
    usb, by_id = tmp_path / 'usb', tmp_path / 'by-id'
    by_id.mkdir()
    for folder, serial in [('2-1', 'follower'), ('2-2', 'leader')]:
        dev = usb / folder
        dev.mkdir(parents=True)
        for name, value in {'idVendor': '1a86', 'idProduct': '7523', 'serial': serial, 'busnum': '2', 'devnum': folder[-1]}.items():
            (dev / name).write_text(value)
        tty = dev / 'ttyUSB0' if serial == 'follower' else dev / 'ttyUSB1'
        tty.mkdir()
        (by_id / f'usb-{serial}').symlink_to('/dev/' + tty.name)
    found = discover_arm(usb, by_id)
    assert {i['serial'] for i in found} == {'follower', 'leader'}
    follower = next(i for i in found if i['serial'] == 'follower')
    assert follower['serial_devices'] == [{'path': str(by_id / 'usb-follower'), 'device': '/dev/ttyUSB0'}]


class MovingBrick(Brick):
    def __init__(self):
        super().__init__()
        self.stalled = False
        self.feeder_at_stop = False
        self.touch = 1.

    def write(self, endpoint, packet, timeout):
        result = super().write(endpoint, packet, timeout)
        ops = packet[7:]
        if ops[0] == 0xaf:
            direction = struct.unpack('b', ops[4:5])[0]
            if ops[2] == 8:
                self.touch = float(self.encoders[19] <= 0)
            elif direction < 0 and self.encoders[16] <= -5:
                self.encoders[16] = -5.
        elif ops[0] == 0xb2:
            self.encoders[16 if ops[2] == 1 else 19] = 0.
        return result


def test_home_proves_release_and_approach_then_feeder_stall_backoff():
    c = config()
    c['timeouts']['operation_s'] = 3
    c['ev3']['homing']['backoff_deg'] = 5
    c['ev3']['feeder']['home_backoff_deg'] = 5
    c['ev3']['feeder']['stall_window_s'] = 0.02
    brick = MovingBrick()
    ev3 = OriginalEV3(c, brick)
    ev3.connect()
    ev3.preflight()
    ev3.home()
    assert ev3.homed
    sorter_directions = [struct.unpack('b', p[4:5])[0] for p in brick.motion if p[2] == 8]
    assert sorter_directions[0] > 0 and sorter_directions[-1] < 0
    feeder_directions = [struct.unpack('b', p[4:5])[0] for p in brick.motion if p[2] == 1]
    assert feeder_directions[0] < 0 and feeder_directions[-1] > 0
    assert brick.encoders == {16:0., 19:0.}
    assert brick.stop_masks[-1] == 9


def test_position_and_feed_stop_after_encoder_arrival():
    c = config()
    c['timeouts']['operation_s'] = 1
    c['ev3']['feeder']['travel_deg'] = 5
    brick = Brick()
    brick.stalled = False
    ev3 = OriginalEV3(c, brick)
    ev3.connect()
    ev3.preflight()
    ev3.homed = True
    ev3.position('blue')
    ev3.feed()
    assert brick.encoders[19] == 10 and brick.encoders[16] == 5
    assert brick.stop_masks == [9, 9]


def test_cancellation_stops_without_rearming():
    ev3, brick = ready()
    ev3.homed = True
    original = brick.write
    def cancel_after_slice(endpoint, packet, timeout):
        result = original(endpoint, packet, timeout)
        if packet[7] == 0xaf:
            ev3.cancel.set()
        return result
    brick.write = cancel_after_slice
    with pytest.raises(EV3Error, match='cancel'):
        ev3.feed()
    assert len(brick.motion) == 1 and brick.stop_masks == [9]


def test_tracking_travel_limit_stops():
    ev3, brick = ready()
    ev3.homed = True
    original = brick.write
    def overshoot(endpoint, packet, timeout):
        result = original(endpoint, packet, timeout)
        if packet[7] == 0xaf:
            brick.encoders[16] = 1000
        return result
    brick.write = overshoot
    with pytest.raises(EV3Error, match='travel'):
        ev3.feed()
    assert brick.stop_masks == [9]


def test_touch_stuck_released_prevents_homing():
    ev3, brick = ready()
    brick.stalled = False
    with pytest.raises((TimeoutError, EV3Error)):
        ev3.home()
    assert not ev3.homed and brick.stop_masks == [9]


def test_stop_failure_preserves_original_motion_fault():
    ev3, brick = ready()
    ev3.homed = True
    original = brick.write
    def fail_stop(endpoint, packet, timeout):
        if packet[7] == 0xa3:
            raise EOFError('stop link lost')
        return original(endpoint, packet, timeout)
    brick.write = fail_stop
    with pytest.raises(TimeoutError):
        ev3.feed()
    assert ev3.faulted


def test_mode_mismatch_prevents_admission():
    brick = Brick()
    original = brick.read
    def wrong_mode(endpoint, size, timeout):
        reply = original(endpoint, size, timeout)
        if brick.writes[-1][7:9] == b'\x99\x05':
            return reply[:-1] + b'\x01'
        return reply
    brick.read = wrong_mode
    ev3 = OriginalEV3(config(), brick)
    ev3.connect()
    with pytest.raises(EV3Error, match='mode'):
        ev3.preflight()
    assert not ev3.preflighted


def test_preflight_failure_revokes_previous_admission():
    ev3, brick = ready()
    brick.types[16] = 7
    with pytest.raises(EV3Error):
        ev3.preflight()
    assert not ev3.preflighted


def test_nonfinite_encoder_fault_stops_both():
    ev3, brick = ready()
    ev3.homed = True
    brick.encoders[16] = float('nan')
    with pytest.raises(EV3Error, match='nonfinite'):
        ev3.feed()
    assert brick.stop_masks == [9] and not brick.motion


def test_sorter_stuck_pressed_cannot_move_away_past_backoff():
    c = config()
    c['timeouts']['operation_s'] = 2
    c['ev3']['homing']['backoff_deg'] = 5
    brick = Brick()
    brick.touch = 1.
    brick.stalled = False
    ev3 = OriginalEV3(c, brick)
    ev3.connect()
    ev3.preflight()
    ev3.homed = True
    with pytest.raises(EV3Error, match='switch'):
        ev3.position('green')
    assert brick.stop_masks == [9]


def test_bad_feed_ack_is_one_motion_delivery_then_latched_fault():
    ev3, brick = ready()
    ev3.homed = True
    original = brick.read
    def malformed_ack(endpoint, size, timeout):
        if brick.writes[-1][7] == 0xaf:
            return b'\0'
        return original(endpoint, size, timeout)
    brick.read = malformed_ack
    with pytest.raises(EV3Error, match='length'):
        ev3.feed()
    with pytest.raises(EV3Error, match='fault'):
        ev3.feed()
    assert len(brick.motion) == 1 and brick.stop_masks == [9]


def test_unrelated_counter_then_matching_reply_does_not_resend():
    wire = Wire()
    original = wire.read
    replies = iter([b'\x03\0\xff\xff\x02'])
    def stale_first(endpoint, size, timeout):
        return next(replies, None) or original(endpoint, size, timeout)
    wire.read = stale_first
    assert DirectUSB(wire).command(b'\xa3\0\x09\x01', 0, .05) == b''
    assert len(wire.writes) == 1


@pytest.mark.parametrize('value', [0, 1, 6, 8, 2.5, float('nan')])
def test_unknown_color_is_not_routed(value):
    ev3, brick = ready()
    brick.color = value
    with pytest.raises(EV3Error):
        ev3.read_color()
    assert not brick.motion


def test_invalid_firmware_identity_prevents_admission():
    brick = Brick()
    original = brick.read
    def invalid_identity(endpoint, size, timeout):
        reply = original(endpoint, size, timeout)
        if brick.writes[-1][7] == 0x81:
            return reply[:5] + b'unknown\0' + bytes(24)
        return reply
    brick.read = invalid_identity
    ev3 = OriginalEV3(config(), brick)
    ev3.connect()
    with pytest.raises(EV3Error, match='firmware'):
        ev3.preflight()
    assert not brick.motion and not ev3.preflighted


def test_queued_feed_rechecks_fault_and_home_after_lock(monkeypatch):
    ev3, brick = ready()
    ev3.homed = True
    class QueuedLock:
        released = False
        def acquire(self, timeout):
            assert timeout > 0
            ev3.faulted = True
            ev3.homed = False
            ev3.cancel.set()
            return True
        def release(self):
            self.released = True
    lock = QueuedLock()
    ev3.motion_lock = lock
    with pytest.raises(EV3Error, match='fault'):
        ev3.feed()
    assert not brick.motion
    assert ev3.cancel.is_set() and lock.released


def test_queued_feed_cannot_clear_cancellation():
    ev3, brick = ready()
    ev3.homed = True
    class QueuedLock:
        def acquire(self, timeout):
            ev3.cancel.set()
            return True
        def release(self):
            pass
    ev3.motion_lock = QueuedLock()
    with pytest.raises(EV3Error, match='cancel'):
        ev3.feed()
    assert not brick.motion and ev3.cancel.is_set()


def test_stop_revokes_feed_already_queued_for_motion_lock():
    ev3, brick = ready()
    ev3.homed = True
    class QueuedLock:
        def acquire(self, timeout):
            ev3.stop()
            return True
        def release(self):
            pass
    ev3.motion_lock = QueuedLock()
    with pytest.raises(EV3Error, match='revoked'):
        ev3.feed()
    assert not brick.motion and brick.stop_masks == [9]


def test_new_command_after_completed_stop_is_admitted():
    ev3, brick = ready()
    ev3.homed = True
    brick.stalled = False
    ev3.stop()
    ev3.wait_stopped()
    ev3.position('blue')
    assert brick.encoders[19] == 10
