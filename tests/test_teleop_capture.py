from workshop.teleop_capture import bounded_goal, normalized_position

def test_relative_teleop_has_no_initial_jump_and_limits_fast_input():
    start={'j':10.0}; previous={'j':10.0}
    goal,limited=bounded_goal({'j':50.0},{'j':50.0},start,previous,.008,{'j':(-10,30)},{'j':30})
    assert goal==start and not limited
    goal,limited=bounded_goal({'j':100.0},{'j':50.0},start,previous,.008,{'j':(-10,30)},{'j':30})
    assert abs(goal['j']-10.24)<1e-9 and limited

def test_bounds_and_encoder_units():
    goal,_=bounded_goal({'j':-200},{'j':0},{'j':0},{'j':-9.9},.008,{'j':(-10,10)},{'j':30})
    assert goal['j']==-10
    cal={'range_min':1000,'range_max':3000}
    assert normalized_position(2000,cal,False)==0
    assert normalized_position(2001,cal,False)==360/4095
    assert normalized_position(2000,cal,True)==50

def test_full_calibrated_workspace_has_no_dance_displacement_limit():
    goal,limited=bounded_goal({'j':70},{'j':0},{'j':0},{'j':69.9},.008,{'j':(-120,120)},{'j':60})
    assert goal['j']==70 and not limited
