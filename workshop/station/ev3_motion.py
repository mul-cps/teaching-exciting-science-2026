"""Backend-neutral bounded motion, homing and admission rules."""
import math
import time

class EV3Error(RuntimeError):
    """Identity, protocol or motion safety check failed."""


def _positive(value):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError('deadline must be positive and finite')
    return value


def _remaining(deadline):
    seconds = deadline - time.monotonic()
    if seconds <= 0:
        raise TimeoutError('EV3 operation deadline expired')
    return seconds


def read_stable_color(read_value, deadline):
    """Require three identical supported numeric colors, 50 ms apart.

    Backends can share this admission rule by providing a deadline-bound reader.
    Any unknown/mismatched value faults immediately; no routing or polling retry.
    """
    colors = {2: 'blue', 3: 'green', 4: 'yellow', 5: 'red'}
    previous = None
    for sample in range(3):
        _remaining(deadline)
        value = read_value()
        _remaining(deadline)
        if value not in colors:
            raise EV3Error('unsupported color classification')
        if previous is not None and value != previous:
            raise EV3Error('unstable color classification')
        previous = value
        if sample < 2:
            time.sleep(min(0.05, _remaining(deadline)))
    return colors[previous]


class EV3Motion:
    def _deadline(self, stop=False):
        return time.monotonic() + self.config['timeouts']['stop_s' if stop else 'operation_s']


    def read_color(self):
        if self.faulted:
            raise EV3Error('EV3 fault is latched')
        if not self.preflighted:
            raise EV3Error('preflight required')
        deadline = self._deadline()
        try:
            return read_stable_color(lambda: self._value('color', deadline), deadline)
        except BaseException:
            self.faulted = True
            self.homed = False
            raise


    def stop(self):
        self.stop_generation += 1
        self._stop(self._deadline(stop=True))


    def wait_stopped(self):
        return self._wait_stopped(self._deadline(stop=True))


    def _wait_stopped(self, deadline):
        while True:
            busy = self._busy(deadline)
            if busy == 0:
                return True
            if busy != 1:
                raise EV3Error('invalid motor busy status')
            time.sleep(min(0.01, _remaining(deadline)))


    def _admit(self, deadline, needs_home):
        generation = self.stop_generation
        if not self.motion_lock.acquire(timeout=_remaining(deadline)):
            raise TimeoutError('EV3 motion lock deadline')
        try:
            # State can change while a command waits behind an active move.
            # Check only after acquiring ownership; never reset cancellation.
            if self.faulted:
                raise EV3Error('EV3 fault is latched; create a new backend after inspection')
            if not self.preflighted or needs_home and not self.homed:
                raise EV3Error('preflight and home required')
            if self.cancel.is_set():
                raise EV3Error('EV3 movement cancelled')
            if generation != self.stop_generation:
                raise EV3Error('queued EV3 movement revoked by stop')
            if not self.injected and not (self.config['enabled'] and self.config['commissioned']):
                raise EV3Error('commissioned enabled station required for motion')
            self.operation_generation = generation
        except BaseException:
            self.motion_lock.release()
            raise


    def _cancelled(self):
        return self.cancel.is_set() or self.operation_generation != self.stop_generation


    def _track(self, name, direction, speed, deadline, max_travel, target=None, switch=None, stall_home=False):
        start = self._value(name, deadline)
        progress, progress_time = start, time.monotonic()
        released = False
        feeder = self.ev3['feeder']
        while True:
            if self._cancelled():
                raise EV3Error('EV3 movement cancelled')
            current = self._value(name, deadline)
            if abs(current - start) > max_travel:
                raise EV3Error('EV3 maximum encoder travel exceeded')
            if (current - start) * direction < -self.ev3['tolerance_deg']:
                raise EV3Error('encoder moved opposite commissioned direction')
            touch = self._value('touch', deadline) if name == 'sorter' else None
            if name == 'sorter' and switch is None:
                if touch == 0:
                    released = True
                elif released or (target is not None and abs(current - start) >= self.ev3['homing']['backoff_deg']):
                    raise EV3Error('unexpected or stuck sorter limit switch')
            if switch is not None and touch == switch:
                return current
            if target is not None:
                if abs(current - target) <= self.ev3['tolerance_deg']:
                    return current
                if (current - target) * direction > self.ev3['tolerance_deg']:
                    raise EV3Error('EV3 target overshoot')
                if name == 'sorter' and touch == 1 and direction == -self.ev3['directions']['sorter']:
                    raise EV3Error('unexpected sorter limit switch')
            if abs(current - progress) > feeder['stall_tolerance_deg']:
                progress, progress_time = current, time.monotonic()
            elif time.monotonic() - progress_time >= feeder['stall_window_s']:
                if stall_home:
                    return current
                raise EV3Error('EV3 stalled: encoder lacks progress')
            self._slice(name, direction, speed, deadline)


    def _finish(self, error):
        try:
            deadline = self._deadline(stop=True)
            self._stop(deadline)
            self._wait_stopped(deadline)
        except BaseException as cleanup_error:
            self.faulted = True
            if error is None:
                raise
            error.add_note(f'EV3 movement stop/wait cleanup failed: {cleanup_error}')
        finally:
            self.motion_lock.release()


    def _run(self, callback, needs_home=True):
        deadline = self._deadline()
        self._admit(deadline, needs_home)
        error = None
        try:
            return callback(deadline)
        except BaseException as caught:
            error = caught
            self.faulted = True
            self.homed = False
            raise
        finally:
            self._finish(error)


    def home(self):
        def action(deadline):
            self.homed = False
            homing, feeder = self.ev3['homing'], self.ev3['feeder']
            direction = self.ev3['directions']['sorter']
            end = min(deadline, time.monotonic() + homing['timeout_s'])
            start = self._value('sorter', end)
            # Always move away, even initially pressed; prove released before
            # approaching. No zeroing merely because the initial switch is high.
            self._track('sorter', direction, homing['speed_dps'], end, homing['max_travel_deg'], target=start + direction * homing['backoff_deg'])
            if self._value('touch', end) != 0:
                raise EV3Error('homing switch did not release')
            remaining = homing['max_travel_deg'] - abs(self._value('sorter', end) - start)
            if remaining <= 0:
                raise EV3Error('homing release exhausted travel budget')
            self._track('sorter', -direction, homing['speed_dps'], end, remaining, switch=1)
            self._stop(end)
            self._wait_stopped(end)
            self._zero('sorter', end)
            if feeder['home_travel_deg'] <= 0:
                raise EV3Error('feeder stall homing travel must be commissioned')
            end = min(deadline, time.monotonic() + feeder['home_timeout_s'])
            home_direction = feeder.get('home_direction', 1)
            stalled = self._track('feeder', home_direction, feeder['home_speed_dps'], end, feeder['home_travel_deg'], stall_home=True)
            self._stop(end)
            self._track('feeder', -home_direction, feeder['home_speed_dps'], end, feeder['home_backoff_deg'] + self.ev3['tolerance_deg'], target=stalled - home_direction * feeder['home_backoff_deg'])
            self._stop(end)
            self._wait_stopped(end)
            self._zero('feeder', end)
            self.homed = True
        return self._run(action, needs_home=False)


    def position(self, color):
        if color not in self.ev3['destinations']:
            raise ValueError('unsupported color destination')
        def action(deadline):
            target = self.ev3['directions']['sorter'] * self.ev3['destinations'][color]
            current = self._value('sorter', deadline)
            direction = 1 if target > current else -1
            return self._track('sorter', direction, self.ev3['homing']['speed_dps'], deadline,
                               self.ev3['homing']['max_travel_deg'], target=target)
        return self._run(action)


    def feed(self):
        def action(deadline):
            fdir, feeder = self.ev3['directions']['feeder'], self.ev3['feeder']
            start = self._value('feeder', deadline)
            self._track('feeder', fdir, feeder['speed_dps'], deadline,
                        feeder['travel_deg'] + self.ev3['tolerance_deg'], target=start + fdir * feeder['travel_deg'])
            self._stop(deadline)
            self._wait_stopped(deadline)
            return self._track('feeder', -fdir, feeder['speed_dps'], deadline,
                               feeder['travel_deg'] + self.ev3['tolerance_deg'], target=start)
        return self._run(action)
