"""One failed pick must prevent the next physical dispense."""
import threading

import pytest

from workshop.station.ros_sorting import StableTargetSelector, run_sorting


def status(stamp, *, track=3, x=.12, state="tracking"):
    return {"state": state, "stamp": stamp, "frame_id": "charuco_board",
            "objects": [{"track_id": track, "color": "red",
                         "position_world": [x, .04, .02], "pickup_eligible": True}],
            "grasp_candidates": [{"track_id": track, "pickup_eligible": True}]}


@pytest.mark.parametrize("record", ["objects", "grasp_candidates"])
@pytest.mark.parametrize("eligible", [None, False])
def test_preview_only_track_never_becomes_automatic_sort_target(record, eligible):
    select = StableTargetSelector(after_stamp=10., excluded_ids=set())
    for stamp in (10.1, 10.2, 10.3, 10.4):
        item = status(stamp)
        if eligible is None:
            item[record][0].pop("pickup_eligible")
        else:
            item[record][0]["pickup_eligible"] = eligible
        assert select.update(item, now=stamp) is None


def test_target_requires_three_distinct_fresh_stationary_images():
    select = StableTargetSelector(after_stamp=10., excluded_ids={2})
    assert select.update(status(10.1), now=10.1) is None
    assert select.update(status(10.1), now=10.2) is None
    assert select.update(status(10.2), now=10.2) is None
    result = select.update(status(10.3), now=10.3)
    assert result.track_id == "red-0003"
    assert result.position == (.12, .04, .02)


@pytest.mark.parametrize("bad", [status(9.), status(10.1, track=2),
                                status(10.1, state="registration_unavailable")])
def test_old_existing_or_unlocalized_target_cannot_be_selected(bad):
    select = StableTargetSelector(after_stamp=10., excluded_ids={2})
    for _ in range(4):
        assert select.update(bad, now=10.2) is None


def test_unstable_target_resets_consecutive_image_requirement():
    select = StableTargetSelector(after_stamp=10., excluded_ids=set())
    assert select.update(status(10.1), now=10.1) is None
    assert select.update(status(10.2, x=.13), now=10.2) is None
    assert select.update(status(10.3, x=.13), now=10.3) is None
    assert select.update(status(10.4, x=.13), now=10.4).track_id == "red-0003"


class Devices:
    def __init__(self, *, fail=None, evidence=True):
        self.events = []
        self.fail = fail
        self.evidence = evidence

    def require_ready(self, deadline, cancel):
        self.events.append("ready")
        if self.fail == "ready":
            raise ValueError("calibration missing")

    def wait_clear(self, deadline, cancel):
        self.events.append("clear")

    def dispense(self, deadline, cancel):
        self.events.append("dispense")
        if self.fail == "dispense":
            raise RuntimeError("feeder jam")
        return {"job_id": 4}

    def detect(self, deadline, cancel):
        self.events.append("detect")
        if self.fail == "detect":
            raise TimeoutError("no brick")
        return {"track_id": "red-0003", "color": "red"}

    def pick_place(self, target, destination, deadline, cancel):
        self.events.append("pick_place")
        if self.fail == "pick_place":
            raise RuntimeError("lost localization")
        return {"state": "pick_place_complete", "track_id": target["track_id"],
                "pickup_verified": self.evidence, "lift_verified": self.evidence,
                "release_verified": self.evidence}

    def stop(self):
        self.events.append("stop")


def test_verified_release_precedes_next_dispense():
    devices = Devices()
    log = []
    result = run_sorting(devices, destinations={"red": [0., 0., .1, 0., 0., 0., 1.]},
                         cycles=2, log=log.append)
    assert result == {"completed": 2, "colors": {"red": 2}}
    assert devices.events == ["ready", "clear", "dispense", "detect", "pick_place",
                              "clear", "clear", "dispense", "detect", "pick_place", "clear"]
    assert [item["event"] for item in log].count("cycle_complete") == 2


@pytest.mark.parametrize("failure", ["ready", "dispense", "detect", "pick_place"])
def test_failure_stops_without_admitting_another_brick(failure):
    devices = Devices(fail=failure)
    with pytest.raises((ValueError, RuntimeError, TimeoutError)):
        run_sorting(devices, destinations={"red": [0., 0., .1, 0., 0., 0., 1.]},
                    cycles=20, log=lambda event: None)
    assert devices.events[-1] == "stop"
    assert devices.events.count("dispense") <= 1


def test_planning_or_jaw_success_does_not_admit_second_brick():
    devices = Devices(evidence=False)
    with pytest.raises(RuntimeError, match="verified"):
        run_sorting(devices, destinations={"red": [0., 0., .1, 0., 0., 0., 1.]},
                    cycles=2, log=lambda event: None)
    assert devices.events.count("dispense") == 1
    assert devices.events[-1] == "stop"


def test_operator_cancellation_prevents_dispensing():
    cancel = threading.Event()
    cancel.set()
    devices = Devices()
    with pytest.raises(InterruptedError):
        run_sorting(devices, destinations={"red": [0., 0., .1, 0., 0., 0., 1.]},
                    cycles=20, log=lambda event: None, cancel=cancel)
    assert "dispense" not in devices.events


def test_invalid_destination_rejected_before_device_work():
    devices = Devices()
    with pytest.raises(ValueError):
        run_sorting(devices, destinations={"red": [float("nan")] * 7},
                    cycles=1, log=lambda event: None)
    assert not devices.events
