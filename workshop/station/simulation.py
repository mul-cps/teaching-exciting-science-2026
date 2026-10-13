"""Finite recording backends enforcing clearance and stopped interlocks."""
from .config import COLORS


class SimEV3:
    def __init__(self, events=None, color='blue'):
        self.events = events if events is not None else []
        self.color = color
        self.stopped = True
        self.stopped_confirmed = True
        self.arm = None
        self.stop_error = None
        self.wait_stopped_error = None

    def _record(self, event):
        self.events.append('ev3.' + event)

    def home(self):
        self._require_clear()
        self._record('home')
        self.stopped = self.stopped_confirmed = True

    def read_color(self):
        self._record('read_color')
        return self.color

    def _require_clear(self):
        if self.arm is None or not self.arm.clear_confirmed:
            raise RuntimeError('EV3 movement requires confirmed arm clearance')

    def position(self, color):
        self._require_clear()
        if color not in COLORS:
            raise ValueError('unknown color')
        self._record('position:' + color)
        self.stopped = self.stopped_confirmed = False

    def feed(self):
        self._require_clear()
        self._record('feed')
        self.stopped = self.stopped_confirmed = False

    def stop(self):
        self._record('stop')
        if self.stop_error:
            raise self.stop_error
        self.stopped = True
        self.stopped_confirmed = False

    def wait_stopped(self):
        self._record('wait_stopped')
        if self.wait_stopped_error:
            raise self.wait_stopped_error
        if not self.stopped:
            raise RuntimeError('EV3 has not stopped')
        self.stopped_confirmed = True


class SimArm:
    def __init__(self, events=None, ev3=None):
        self.events = events if events is not None else []
        self.ev3 = ev3
        if ev3 is not None:
            ev3.arm = self
        self.clear_confirmed = False
        self.at_clearance = False
        self.stopped = True
        self.stop_error = None
        self.wait_clear_error = None

    def _record(self, event):
        self.events.append('arm.' + event)

    def _require_stopped(self):
        if self.ev3 is None or not self.ev3.stopped or not self.ev3.stopped_confirmed:
            raise RuntimeError('arm entry requires confirmed EV3 stoppage')

    def home(self):
        self._require_stopped()
        self._record('home')
        self.at_clearance = True
        self.clear_confirmed = False

    def load_piece(self):
        self._require_stopped()
        self._record('load_piece')
        self.stopped = False
        self.at_clearance = self.clear_confirmed = False

    def pick_and_place(self, color):
        self._require_stopped()
        if color not in COLORS:
            raise ValueError('unknown color')
        self._record('pick_and_place:' + color)
        self.stopped = False
        self.at_clearance = self.clear_confirmed = False

    def clear(self):
        self._record('clear')
        self.at_clearance = True
        self.clear_confirmed = False

    def wait_clear(self):
        self._record('wait_clear')
        if self.wait_clear_error:
            raise self.wait_clear_error
        if not self.at_clearance:
            raise RuntimeError('arm has not reached clearance')
        self.clear_confirmed = True
        self.stopped = True

    def stop(self):
        self._record('stop')
        if self.stop_error:
            raise self.stop_error
        self.stopped = True
