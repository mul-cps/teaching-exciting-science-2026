"""Allowlisted perception-unit jobs; unknown OS outcomes retain a durable barrier."""
import json
import hashlib
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid


PROPERTIES = ("Id", "LoadState", "ActiveState", "SubState", "Job", "InvocationID", "StateChangeTimestampMonotonic")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def read_unit(unit):
    try:
        result = subprocess.run(["systemctl", "--user", "show", "--no-pager", "--property=" + ",".join(PROPERTIES), "--", unit],
                                text=True, capture_output=True, timeout=3, check=False)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("systemd unit inspection timed out; outcome unknown") from error
    if result.returncode:
        raise RuntimeError("could not inspect configured unit: " + result.stderr[-1000:])
    row = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if set(row) != set(PROPERTIES) or row["Id"] != unit or row["LoadState"] != "loaded":
        raise RuntimeError("unit identity or complete systemd state is unavailable")
    if not row["StateChangeTimestampMonotonic"].isdigit():
        raise RuntimeError("unit state-change timestamp is unavailable")
    if row["Job"] == "":
        row["Job"] = "0"
    return row


class OperatorRestartMixin:
    def _operator_restart_init(self):
        self._operator_restart_lock = threading.RLock()
        self._operator_external_job = None
        self._operator_restart_result = None
        self._operator_restart_thread = None
        self._operator_restart_source_sha256 = {
            name: hashlib.sha256(Path(__file__).with_name(name + ".py").read_bytes()).hexdigest()
            for name in ("operator_restart", "trajectory_server")}
        configured = list(self.get_parameter("operator_service_units").value)
        self._operator_restart_units = {value for value in configured if value}
        for unit in self._operator_restart_units:
            if (not re.fullmatch(r"so101-[A-Za-z0-9_.@-]+\.service", unit) or
                    re.search(r"arm|bridge|move.group|rviz|teleop|ev3", unit, re.IGNORECASE)):
                raise ValueError("operator restart allowlist accepts perception user units only")
        self._operator_station_reset_units = {value for value in self.get_parameter("operator_station_reset_units").value if value}
        for unit in self._operator_station_reset_units:
            if (not re.fullmatch(r"so101-[A-Za-z0-9_.@-]+\.service", unit) or
                    re.search(r"arm|bridge|rviz|teleop|ev3", unit, re.IGNORECASE)):
                raise ValueError("station reset allowlist accepts nonactuating station user units only")
        self._operator_restart_journal = Path(str(self.get_parameter("operator_maintenance_journal").value))
        if not self._operator_restart_journal.is_absolute():
            raise ValueError("operator maintenance journal must be absolute")

    def _operator_restart_load_pending(self):
        if self._operator_restart_journal.exists():
            try:
                record = json.loads(self._operator_restart_journal.read_text())
                self._operator_restart_validate_record(record)
            except (ValueError, TypeError, RuntimeError, OSError) as error:
                # Invalid evidence blocks admission; never silently delete it or leak an armed startup.
                record = {"invalid_pending_journal": str(error)}
            self._operator_external_job = record
            self._operator_maintenance_token = record.get("token")
            self._operator_maintenance_deadline = None
            self._active_goal = "operator_maintenance_unknown"
            self._stop_latched = True
            self._mark_owner_fault("pending_external_restart_journal")

    def _operator_restart_validate_record(self, record):
        if (not isinstance(record, dict) or set(record) != {"schema", "job_id", "token", "owner_generation", "arm_id", "calibration_sha256",
                "owner_source_sha256", "restart_source_sha256", "unit", "requested_monotonic_us", "boot_id", "before"} or
                record.get("schema") != "operator-restart-pending-v1" or
                any(not isinstance(record.get(k), str) or len(record[k]) != 32 or
                    any(c not in "0123456789abcdef" for c in record[k]) for k in ("job_id", "token")) or
                record.get("unit") not in (self._operator_restart_units | self._operator_station_reset_units) or
                record.get("arm_id") != self._expected_arm_id or record.get("calibration_sha256") != self._expected_calibration_sha256 or
                record.get("owner_source_sha256") != self._owner_source_expected or
                record.get("restart_source_sha256") != self._operator_restart_source_sha256 or
                type(record.get("owner_generation")) not in (int, float) or not math.isfinite(record["owner_generation"]) or record["owner_generation"] <= 0 or
                type(record.get("requested_monotonic_us")) is not int or record["requested_monotonic_us"] < 0 or
                not isinstance(record.get("boot_id"), str) or not re.fullmatch(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}", record["boot_id"]) or
                not isinstance(record.get("before"), dict) or set(record["before"]) != set(PROPERTIES) or
                any(not isinstance(v, str) for v in record["before"].values()) or
                record["before"].get("Id") != record.get("unit") or record["before"].get("LoadState") != "loaded" or
                not record["before"].get("StateChangeTimestampMonotonic", "").isdigit() or
                not re.fullmatch(r"(?:[0-9a-f]{32})?", record["before"].get("InvocationID", ""))):
            raise RuntimeError("operator restart journal identity/token/unit/cohort is invalid; explicit frozen-cohort recovery required")

    def _operator_restart_write(self, path, record):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
        try:
            with temporary.open("x") as stream:
                json.dump(record, stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(path)
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            temporary.unlink(missing_ok=True)

    def _operator_restart_begin(self, token, unit):
        with self._operator_restart_lock:
            allowed = self._operator_restart_units
            if getattr(self, "_operator_maintenance_purpose", None) == "station_reset":
                allowed = allowed | self._operator_station_reset_units
            if unit not in allowed or self._operator_external_job is not None:
                raise RuntimeError("restart unit is not allowlisted or another external job is unresolved")
            if token != self._operator_maintenance_token or not self._active_goal_is("operator_maintenance"):
                raise RuntimeError("restart does not own a live maintenance reservation")
            before = read_unit(unit)
            if before["Job"] != "0":
                raise RuntimeError("unit already has an external manager job; no restart submitted")
            with self._active_lock:
                state = self._cached_owner_state
                now = time.monotonic()
                if (self._closed or self._stop_latched or self._owner_faulted or
                        token != self._operator_maintenance_token or not self._active_goal_is("operator_maintenance") or
                        self._process is None or self._process.poll() is not None or
                        self._cached_owner_generation != self._owner_started or
                        not isinstance(state, dict) or state.get("arm_id") != self._expected_arm_id or
                        state.get("calibration_sha256") != self._expected_calibration_sha256 or
                        state.get("owner_source_sha256") != self._owner_source_expected or
                        state.get("torque") != {j: 0 for j in JOINTS} or
                        any(type(state.get(key)) not in (int, float) or not self._owner_started <= state[key] <= now or
                            now-state[key] > .25 for key in ("monotonic", "torque_monotonic"))):
                    raise RuntimeError("maintenance admission changed before external restart dispatch")
                record = dict(schema="operator-restart-pending-v1", job_id=uuid.uuid4().hex, token=token,
                    owner_generation=self._owner_started, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256, owner_source_sha256=dict(self._owner_source_expected),
                    restart_source_sha256=dict(self._operator_restart_source_sha256),
                    unit=unit, requested_monotonic_us=time.monotonic_ns()//1000,
                    boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(), before=before)
                self._operator_restart_write(self._operator_restart_journal, record)
                self._operator_external_job = record
                self._operator_restart_result = None
                thread = threading.Thread(target=self._operator_restart_run, args=(record,), name="station-perception-restart", daemon=True)
                self._operator_restart_thread = thread
                thread.start()
                return record["job_id"]

    def _operator_restart_completion(self, record):
        after = read_unit(record["unit"])
        if after["Job"] != "0":
            raise RuntimeError("systemd manager job is still pending")
        same_boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip() == record["boot_id"]
        transitioned = (not same_boot or int(after["StateChangeTimestampMonotonic"]) >= record["requested_monotonic_us"])
        active = after["ActiveState"] == "active" and after["SubState"] in {"running", "exited"}
        failed = after["ActiveState"] in {"failed", "inactive"} and after["SubState"] in {"failed", "dead"}
        if active and (not transitioned or after["InvocationID"] == record["before"].get("InvocationID")):
            raise RuntimeError("active unit has no proven new invocation/transition after the restart request")
        if not active and not failed:
            raise RuntimeError("unit has no proven terminal manager state")
        return {"job_id": record["job_id"], "unit": record["unit"], "success": active,
                "kind": "restart_unit_terminal", "before": record["before"], "after": after,
                "observed_monotonic": time.monotonic(), "owner_generation": record["owner_generation"]}

    def _operator_restart_finish(self, record, proof):
        with self._operator_restart_lock:
            if self._operator_external_job != record or json.loads(self._operator_restart_journal.read_text()) != record:
                raise RuntimeError("external restart journal/token was replaced")
            archive = self._operator_restart_journal.with_name(self._operator_restart_journal.name + ".resolved-" + record["job_id"])
            self._operator_restart_write(archive, {"pending": record, "proof": proof})
            self._operator_restart_journal.unlink()
            directory = os.open(self._operator_restart_journal.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
            self._operator_external_job = None
            self._operator_restart_result = proof

    def _operator_restart_run(self, record):
        try:
            # No lock is held during OS work. STOP can disarm and retain the unknown barrier.
            result = subprocess.run(["systemctl", "--user", "restart", "--", record["unit"]],
                                    capture_output=True, text=True, timeout=15, check=False)
            proof = self._operator_restart_completion(record)
            proof.update(systemctl_returncode=result.returncode, stderr=result.stderr[-1000:])
            if not proof["success"]:
                self._operator_restart_result = proof
                with self._active_lock:
                    self._active_goal = "operator_maintenance_unknown"
                    self._stop_latched = True
                    self._mark_owner_fault("external_restart_known_failed")
                return  # explicit read-only verification resolves a known failed terminal outcome
            self._operator_restart_finish(record, proof)
        except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
            self._operator_restart_result = {"job_id": record["job_id"], "unit": record["unit"],
                                             "success": False, "kind": "restart_outcome_unknown", "error": str(error)}
            with self._active_lock:
                self._active_goal = "operator_maintenance_unknown"
                self._stop_latched = True
                self._mark_owner_fault("external_restart_outcome_unknown")

    def _operator_restart_verify(self, token):
        with self._operator_restart_lock:
            record = self._operator_external_job
            if record is None or token != record["token"]:
                raise RuntimeError("no unresolved external restart owned by that token")
            if self._operator_restart_thread is not None and self._operator_restart_thread.is_alive():
                raise RuntimeError("restart worker is still active; verification cannot race its OS command")
            self._operator_restart_validate_record(record)
            proof = self._operator_restart_completion(record)
            self._operator_restart_finish(record, proof)
            return proof
