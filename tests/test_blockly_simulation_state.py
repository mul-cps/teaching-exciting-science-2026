import threading
from workshop.blockly_program import SimulationBackend

def test_conveyor_and_brick_follow_commands_not_logs():
    sim=SimulationBackend({'demo':{}})
    def command(**row): sim.command(row,threading.Event(),float('inf'))
    command(op='ev3',command='eject')
    assert sim.status()['simulation_state']['brick']=='belt'
    command(op='ev3',command='left_end')
    state=sim.status()['simulation_state']
    assert state['conveyor_position']=='left'
    assert state['brick']=='pickup'
    command(op='jaw',gap_mm=0)
    command(op='replay',name='demo',speed=.5)
    state=sim.status()['simulation_state']
    assert state['grasp_verified'] is False
    assert state['brick']=='pickup'
    assert state['placed_count']==0
    before=state.copy();sim.log('irrelevant')
    assert sim.status()['simulation_state']==before
