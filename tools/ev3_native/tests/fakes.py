class Port:
    D = 'D'
    A = 'A'
class Stop:
    BRAKE = 'BRAKE'
class Control:
    def __init__(self, motor):
        self.motor = motor
    def limits(self, **kwargs):
        self.motor.runner.calls.append(('limits', kwargs))
    def stalled(self):
        return self.motor.runner.fault == 'stall'
    def done(self):
        return self.motor.target == self.motor.position or self.motor.runner.fault == 'done_error'
class Motor:
    def __init__(self, runner, port):
        self.runner = runner
        self.port = port
        self.position = 0
        self.target = None
        self.control = Control(self)
    def angle(self):
        return self.position
    def reset_angle(self, value):
        self.position = value
        self.runner.calls.append(('reset', self.port, value))
    def brake(self):
        self.runner.calls.append(('brake', self.port))
        self.target = None
    def run_angle(self, speed, angle, then, wait):
        self.runner.calls.append(('run_angle', speed, angle, then, wait))
        self.target = self.position + angle
    def run_target(self, speed, angle, then, wait):
        self.runner.calls.append(('run_target', speed, angle, then, wait))
        self.target = angle
class Speaker:
    def __init__(self, runner):
        self.runner = runner
    def beep(self, duration):
        self.runner.calls.append(('beep', duration))
class Runner:
    Port = Port
    Stop = Stop
    def __init__(self, fault=None):
        self.fault = fault
        self.now = 0
        self.clock = self
        self.calls = []
        self.logs = []
        self.speaker = Speaker(self)
        self.cleaned = False
    def time(self):
        return self.now
    def wait(self, ms):
        self.now += ms
        if hasattr(self, 'a') and self.a.target is not None:
            if self.fault == 'deadline':
                return
            delta = self.a.target - self.a.position
            step = min(2, abs(delta)) * (1 if delta > 0 else -1)
            if self.fault == 'jump':
                step = 26
            elif self.fault == 'wrong':
                step = -4
            elif self.fault == 'budget':
                step = 2
                self.a.target = 200
            self.a.position += step
            if self.fault == 'drift':
                self.d.position += 4
    def log(self, phase, **kwargs):
        self.logs.append((phase, kwargs))
    safe_log = log
    def init_motor(self, port, label):
        self.calls.append(('init', port, label))
        return Motor(self, port)
    def home_d(self):
        self.calls.append(('home_d',))
        self.d.position = 0
    def home_a(self):
        raise AssertionError('No feeder homing allowed')
    def move(self, motor, phase, target, **kwargs):
        self.calls.append(('move', motor.port, target, kwargs))
        motor.position = target
    def stopped(self, motor, phase):
        self.calls.append(('stopped', motor.port, phase))
        motor.brake()
        if self.fault == 'settle_error' and phase == 'A_probe_forward':
            motor.position += 4
        return motor.angle()
    def cleanup(self):
        self.calls.append(('cleanup',))
        self.d.brake()
        if hasattr(self, 'a'):
            self.a.brake()
        self.cleaned = True
