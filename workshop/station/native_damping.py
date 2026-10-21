"""Default-off STS3215 native damping, with a bounded manual lease and RAM journal.

Torque-switch value 2 is documented by Waveshare. Configuration is an explicit
firmware capability declaration, not evidence that this robot is qualified.
No position goal, PID, lock or EEPROM register is written here.
"""
import json
import os
from pathlib import Path
import uuid

from .config import JOINTS
from . import arm as arm_module


class NativeDampingPreflightError(ValueError):
    pass


class NativeDampingMixin:
    def _native_damping_init(self):
        settings = self.arm.settings
        self._native_damping_enabled = settings.get("enable_native_damping", False)
        self._native_damping_restore_only = settings.get("restore_native_damping_only", False)
        if type(self._native_damping_enabled) is not bool or type(self._native_damping_restore_only) is not bool:
            raise ValueError("native damping capability and restore-only flags must be Boolean")
        self._native_damping_limit = settings.get("native_damping_torque_limit", 25)
        if type(self._native_damping_limit) is not int or not 1 <= self._native_damping_limit <= 50:
            raise ValueError("native damping torque limit must be in 1..50 raw /1000")
        self._native_damping_firmware = settings.get("native_damping_firmware", {})
        if self._native_damping_firmware and (
                not isinstance(self._native_damping_firmware, dict) or set(self._native_damping_firmware) != set(JOINTS) or
                any(not isinstance(v, list) or len(v) != 2 or
                    any(type(n) is not int or not 0 <= n <= 255 for n in v)
                    for v in self._native_damping_firmware.values())):
            raise ValueError("native_damping_firmware requires exact six-joint [major, minor] versions")
        self._native_damping = None
        self._native_damping_record = None
        self._native_damping_last_raw = None
        self._native_damping_previous_stamp = None
        self._native_damping_health_at = 0.
        self._native_damping_journal = Path(settings["calibration"]["path"]).resolve().with_suffix(".native-damping-pending.json")

    def _native_damping_identity(self):
        from .discovery import discover_arm
        from .recovery_types import owner_source_sha256
        if self._observe_only or not self.bus.is_connected:
            raise NativeDampingPreflightError("damping requires an ordinary connected owner")
        self._calibration_guard()
        if owner_source_sha256(Path(__file__).parent) != self._owner_source_sha256:
            raise NativeDampingPreflightError("native damping owner source changed")
        usb = self.arm.settings["usb"]
        matches = [row for row in discover_arm() if all(row.get(k) == usb[k] for k in ("vendor_id", "product_id", "serial"))]
        if (not usb["serial"] or len(matches) != 1 or len(matches[0]["serial_devices"]) != 1 or
                self.arm.port is None or Path(matches[0]["serial_devices"][0]["path"]).resolve() != Path(self.arm.port).resolve()):
            raise NativeDampingPreflightError("native damping USB/sole-owner identity mismatch")
        if any(self.bus.motors[j].id != index or self.bus.motors[j].model != "sts3215" for index, j in enumerate(JOINTS, 1)):
            raise NativeDampingPreflightError("native damping requires exact six STS3215 identities")
        major, minor = self._read("Firmware_Major_Version"), self._read("Firmware_Minor_Version")
        return {j: [major[j], minor[j]] for j in JOINTS}

    def _native_damping_validate_record(self, record):
        if (not isinstance(record, dict) or set(record) != {"schema", "trial_id", "arm_id", "calibration_sha256", "usb",
                "owner_source_sha256", "owner_session_id", "created_utc", "firmware", "original", "damping_limit"} or
                record.get("schema") != "native-damping-pending-v1" or
                record.get("arm_id") != self.arm.settings["id"] or record.get("calibration_sha256") != self.calibration_digest or
                record.get("usb") != {k: self.arm.settings["usb"][k] for k in ("vendor_id", "product_id", "serial")} or
                record.get("owner_source_sha256") != self._owner_source_sha256 or
                not isinstance(record.get("trial_id"), str) or len(record["trial_id"]) != 32 or
                any(c not in "0123456789abcdef" for c in record["trial_id"]) or
                type(record.get("damping_limit")) is not int or not 1 <= record["damping_limit"] <= 50):
            raise RuntimeError("pending damping journal identity/schema/source invalid")
        original = record.get("original")
        if not isinstance(original, dict) or set(original) != {"Torque_Limit", "Lock"}:
            raise RuntimeError("pending damping RAM snapshot invalid")
        for register, upper in (("Torque_Limit", 1000), ("Lock", 1)):
            row = original[register]
            if not isinstance(row, dict) or set(row) != set(JOINTS) or any(type(v) is not int or not 0 <= v <= upper for v in row.values()):
                raise RuntimeError("pending damping RAM snapshot outside fixed limits")

    def _native_damping_startup(self):
        if self._native_damping_journal.exists():
            if not self._native_damping_restore_only:
                raise RuntimeError("pending native damping journal blocks startup; use restore_native_damping_only=true")
            record = json.loads(self._native_damping_journal.read_text())
            self._native_damping_validate_record(record)
            if self._native_damping_identity() != record["firmware"]:
                raise RuntimeError("pending damping firmware differs from attached robot")
            self._native_damping_record = record
            for joint in JOINTS:
                self.bus.write("Torque_Enable", joint, 0, normalize=False, num_retry=0)
            if any(self._read("Torque_Enable").values()):
                raise RuntimeError("restore-only damping startup failed to establish all six torque OFF")
            self._native_damping_restore()
        elif self._native_damping_restore_only:
            raise RuntimeError("restore-only damping startup requires a pending journal")

    def _native_damping_enter(self, command):
        if (set(command) != {"command", "request_id"} or not isinstance(command.get("request_id"), str) or
                len(command["request_id"]) != 32 or any(c not in "0123456789abcdef" for c in command["request_id"])):
            raise NativeDampingPreflightError("native damping requires a correlated request UUID")
        if (self._native_damping_enabled is not True or not self._native_damping_firmware or self._native_damping_restore_only or
                self._native_damping is not None or self._native_damping_record is not None or self._native_damping_journal.exists() or
                self.saved_ram is not None or
                self._teleop is not None or self._active_leg is not None or self._failed_endpoint_hold is not None or
                self._component_scope or self._component_saved_ram is not None):
            raise NativeDampingPreflightError("native damping capability/idle admission unavailable")
        firmware = self._native_damping_identity()
        if firmware != self._native_damping_firmware:
            raise NativeDampingPreflightError("native damping installed firmware is not explicitly allowed")
        raw, torque = self._read("Present_Position"), self._read("Torque_Enable")
        if any(torque.values()) or self._arm_bounds_observation(raw, self.clock())["classification"] != "normal":
            raise NativeDampingPreflightError("native damping requires all-six OFF and normal admitted bounds")
        original = {register: self._read(register) for register in ("Torque_Limit", "Lock")}
        record = dict(schema="native-damping-pending-v1", trial_id=uuid.uuid4().hex, arm_id=self.arm.settings["id"],
            calibration_sha256=self.calibration_digest, usb={k: self.arm.settings["usb"][k] for k in ("vendor_id", "product_id", "serial")},
            owner_source_sha256=dict(self._owner_source_sha256), owner_session_id=self._owner_session_id,
            created_utc=self.wall_clock().isoformat(), firmware=firmware, original=original, damping_limit=self._native_damping_limit)
        self._native_damping_validate_record(record)
        self._poll_busy_input()
        with self._native_damping_journal.open("x") as stream:
            json.dump(record, stream, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        self._native_damping_fsync_directory()
        self._native_damping_record = record
        self._shutdown_complete = False
        try:
            for joint in JOINTS:
                self._poll_busy_input()
                self.bus.write("Torque_Limit", joint, self._native_damping_limit, normalize=False, num_retry=0)
            if self._read("Torque_Limit") != {j: self._native_damping_limit for j in JOINTS} or self._read("Lock") != original["Lock"]:
                raise RuntimeError("native damping bounded RAM readback differs")
            if any(self._read("Torque_Enable").values()):
                raise RuntimeError("torque changed before native damping enable")
            # Explicit phase precedes a possibly applied write; cleanup retains the journal on unknown delivery.
            now = self.clock()
            self._native_damping = dict(phase="active", request_id=command["request_id"], deadline_monotonic=now+60.,
                                       entered_monotonic=now, torque_limit=self._native_damping_limit)
            self._native_damping_last_raw = raw
            self._native_damping_previous_stamp = None
            self._native_damping_health_at = 0.
            for joint in JOINTS:
                self._poll_busy_input()
                self.bus.write("Torque_Enable", joint, 2, normalize=False, num_retry=0)
            if self._read("Torque_Enable") != {j: 2 for j in JOINTS}:
                raise RuntimeError("installed firmware did not confirm all-six native damping value 2")
            state = self.publish_state(event="native_damping_enabled")
            return {"ok": True, "command": command["command"], "request_id": command["request_id"], "state": state}
        except BaseException:
            self.shutdown(reason="native_damping_entry_failed")
            raise

    def _native_damping_inventory(self, command):
        if set(command) != {"command"} or self._native_damping is not None:
            raise NativeDampingPreflightError("native damping inventory requires an ordinary idle owner")
        firmware = self._native_damping_identity()
        return {"ok": True, "command": "native_damping_inventory", "firmware": firmware,
                "capability_enabled": self._native_damping_enabled,
                "firmware_allowed": firmware == self._native_damping_firmware,
                "physically_qualified": False, "state": self.publish_state(event="native_damping_inventory")}

    def _native_damping_renew(self, command):
        if set(command) != {"command", "request_id"} or self._native_damping is None:
            raise NativeDampingPreflightError("native damping renewal requires an active manual lease")
        request_id = command.get("request_id")
        if not isinstance(request_id, str) or len(request_id) != 32 or any(c not in "0123456789abcdef" for c in request_id):
            raise NativeDampingPreflightError("native damping renewal requires a correlated UUID")
        if self._native_damping_identity() != self._native_damping_record["firmware"]:
            raise RuntimeError("native damping renewal firmware changed")
        self.publish_state(event="native_damping_renewal_preflight")
        now = self.clock()
        self._native_damping = dict(self._native_damping, request_id=request_id, entered_monotonic=now, deadline_monotonic=now+60.)
        return {"ok": True, "command": command["command"], "request_id": request_id,
                "state": self.publish_state(event="native_damping_renewed")}

    def _native_damping_monitor(self, raw, torque, position_stamp):
        phase = self._native_damping
        if phase is None:
            return
        now = self.clock()
        if now >= phase["deadline_monotonic"] or now-position_stamp > .1 or torque != {j: 2 for j in JOINTS}:
            raise RuntimeError("native damping lease, feedback or torque verification failed")
        if self._native_damping_previous_stamp is not None and position_stamp-self._native_damping_previous_stamp > .1:
            raise RuntimeError("native damping feedback gap exceeded 100 ms")
        self._native_damping_previous_stamp = position_stamp
        if self._arm_bounds_observation(raw, position_stamp)["classification"] != "normal":
            raise RuntimeError("native damping moved outside admitted bounds")
        previous = self._native_damping_last_raw
        if previous is not None and any(abs(raw[j]-previous[j]) > (150 if j == "gripper" else 80) for j in JOINTS):
            raise RuntimeError("native damping manual movement exceeded per-sample bounds")
        self._native_damping_last_raw = raw
        if now-self._native_damping_health_at >= .5:
            temperature, current, status = self._read("Present_Temperature"), self._read("Present_Current"), self._read("Status")
            if any(not 0 <= temperature[j] < 50 or not 0 <= current[j] < 150 or status[j] != 0 for j in JOINTS):
                raise RuntimeError("native damping temperature/current/status guard failed")
            self._native_damping_health_at = self.clock()

    def _native_damping_restore(self):
        record = self._native_damping_record
        if record is None:
            return
        self._native_damping_validate_record(record)
        if json.loads(self._native_damping_journal.read_text()) != record:
            raise RuntimeError("pending native damping journal changed")
        if self._native_damping_identity() != record["firmware"] or any(self._read("Torque_Enable").values()):
            raise RuntimeError("native damping restore requires original firmware and all-six OFF")
        if self._read("Lock") != record["original"]["Lock"]:
            raise RuntimeError("native damping lock state changed; no lock writes permitted")
        for joint, value in record["original"]["Torque_Limit"].items():
            self.bus.write("Torque_Limit", joint, value, normalize=False, num_retry=0)
        restored = {register: self._read(register) for register in ("Torque_Limit", "Lock")}
        if restored != record["original"] or any(self._read("Torque_Enable").values()):
            raise RuntimeError("native damping RAM restoration or all-OFF readback differs")
        archive = self._native_damping_journal.with_name(self._native_damping_journal.name + ".restored-" + record["trial_id"])
        arm_module.atomic_json(archive, dict(record, restored_utc=self.wall_clock().isoformat(), restored_readback=restored))
        self._native_damping_journal.unlink()
        self._native_damping_fsync_directory()
        self._native_damping_record = None
        self._native_damping = None
        self._event("native_damping_restored", trial_id=record["trial_id"], readback=restored)

    def _native_damping_fsync_directory(self):
        directory = os.open(self._native_damping_journal.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
