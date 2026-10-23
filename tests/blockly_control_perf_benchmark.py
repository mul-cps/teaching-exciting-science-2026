"""Cold admission/cancellation microbenchmark; never connects ROS or hardware."""
import json
import threading
import time
import timeit
from tools.run_blockly_station import Application, _ControlCancellation


def main():
    app = Application.__new__(Application)
    app.admission_session = 'a' * 32
    app.admission_epoch = 7
    app.stop_pending = 0
    app.control_cancel = threading.Event()
    stamp = {'session': app.admission_session, 'epoch': app.admission_epoch}
    local, parent = threading.Event(), threading.Event()
    combined = _ControlCancellation(local, parent)
    n = 100000
    measured = {
        'scope': 'cold real admission/control cancellation; no Application initialization, ROS or hardware',
        'iterations': n,
        'check_admission_us': timeit.timeit(lambda: app._check_admission(stamp), number=n) / n * 1e6,
        'begin_control_us': timeit.timeit(lambda: app._begin_control(stamp), number=n) / n * 1e6,
        'two_event_is_set_us': timeit.timeit(combined.is_set, number=n) / n * 1e6,
        'zero_wait_us': timeit.timeit(lambda: combined.wait(0), number=n) / n * 1e6,
    }
    parent.set()
    measured['already_cancelled_wait_us'] = timeit.timeit(lambda: combined.wait(1), number=n) / n * 1e6
    parent.clear()
    cpu = time.process_time(); wall = time.monotonic()
    combined.wait(.05)
    measured['sleep_wait_wall_ms'] = (time.monotonic() - wall) * 1000
    measured['sleep_wait_cpu_ms'] = (time.process_time() - cpu) * 1000
    print(json.dumps(measured))


if __name__ == '__main__':
    main()
