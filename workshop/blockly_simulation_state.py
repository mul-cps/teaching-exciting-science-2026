"""Invented teaching state. No sensor or physical grasp evidence is inferred."""
from dataclasses import dataclass, asdict

@dataclass
class SimulationState:
    conveyor_position: str = 'home'
    brick: str = 'feeder'
    jaw_gap_mm: float = 30
    segment: str = 'idle'
    progress: float = 0
    dispensed_count: int = 0
    placed_count: int = 0
    grasp_verified: bool = False
    assumption: str = 'Synthetic diagram; jaw closure and replay do not establish grasp.'

    def apply(self,row):
        op=row['op'];self.segment=op;self.progress=1
        if op=='ev3':
            command=row['command'];self.segment='conveyor:'+command
            if command=='eject':self.brick='belt';self.dispensed_count+=1
            if command in {'home','left_end','right_end'}:
                self.conveyor_position={'home':'home','left_end':'left','right_end':'right'}[command]
                if self.brick in {'belt','pickup'}:self.brick='belt' if command=='home' else 'pickup'
        elif op=='jaw':self.jaw_gap_mm=row['gap_mm']
        elif op in {'approach_recording','replay'}:self.segment=op+':'+row['name']
        elif op=='sort_cycle':
            self.brick='place';self.placed_count+=1
            self.assumption='Synthetic commissioned cycle assumes successful transfer; no physical grasp evidence.'
        elif op=='stop':self.segment='stopped';self.progress=0

    def snapshot(self): return asdict(self)
