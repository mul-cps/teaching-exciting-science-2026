"""Simple CPS loop. Simulation is the default; no robot can move by accident."""
from dataclasses import dataclass, field


@dataclass
class SimBelt:
    steps: int = 0

    def advance(self):
        self.steps += 1
        print("BELT: advance, then stop")

    def stop(self):
        print("BELT: stopped")


@dataclass
class SimCamera:
    colors: list[str] = field(default_factory=lambda: ["red", "blue", "red", "unknown", "blue"])
    index: int = 0

    def detect(self):
        if not self.colors:
            raise ValueError("SimCamera needs at least one color reading")
        value = self.colors[self.index % len(self.colors)]
        self.index += 1
        print(f"CAMERA: {value}")
        return value


@dataclass
class SimRobot:
    actions: list[str] = field(default_factory=list)

    def sort(self, destination):
        self.actions.append(destination)
        print(f"ROBOT: move object to {destination}")


@dataclass
class Factory:
    belt: object
    camera: object
    robot: object
    counts: dict[str, int] = field(default_factory=lambda: {"left": 0, "right": 0, "reject": 0})

    def one_cycle(self, rules=None):
        rules = {"red": "left", "blue": "right"} if rules is None else rules
        try:
            self.belt.advance()
            self.belt.stop()  # stationary pick; never sense/grasp on a running belt
            color = self.camera.detect()
            destination = rules.get(color, "reject") if isinstance(color, str) else "reject"
            if destination not in ("left", "right", "reject"):
                destination = "reject"
            self.robot.sort(destination)
            self.counts[destination] = self.counts.get(destination, 0) + 1
            return destination
        finally:
            self.belt.stop()
