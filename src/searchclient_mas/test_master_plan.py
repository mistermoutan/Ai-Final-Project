from master_plan import MasterPlan
import test_utilities as util
from action import move,push,pull,north,east,west,south, NOOP


agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
agt2 = util.agent(2, "red")
box0 = util.box("A", "red")

level = [
    [False,False,False,False,False],
    [False,agt0,True,agt2,False],
    [False,True,True,True,False],
    [False,box0,True,agt1,False],
    [False,False,False,False,False]
    ]
initial_state = util.make_state(level)

def test_merge_plan_into_emtpy_master_works():
    agent_id = 0
    p = MasterPlan(3, initial_state)
    result = p.merge_plan_into_master(agent_id, [move(east), move(south)])
    assert result and \
        p.get_plan_of_agent(agent_id) == [move(east), move(south)] and \
        p.get_plan_of_agent(1) == [] and \
        p.get_plan_of_agent(2) == []

def test_merge_illegal_plan_into_master_returns_false():
    p = MasterPlan(3, initial_state)
    #agent 0 crashes into wall by moving west
    result = p.merge_plan_into_master(0, [move(west)])
    assert not result

def test_two_independent_plans_remains_the_same_after_merged_into_master():
    p = MasterPlan(3, initial_state)
    plan_0 = [move(east), move(south), move(west)]
    plan_1 = [move(north), NOOP, NOOP, move(west)]
    result_1 = p.merge_plan_into_master(0, plan_0)
    result_2 = p.merge_plan_into_master(1, plan_1)
    assert result_1 and result_2 and \
        p.get_plan_of_agent(0) == plan_0 and \
        p.get_plan_of_agent(1) == plan_1 and \
        p.get_plan_of_agent(2) == []

def test_two_dependent_plans_merged_into_master_the_last_one_changes():
    p = MasterPlan(3, initial_state)
    plan_0 = [move(east), move(south), move(west)]
    #This plan is the same as the one in the previous test, except without the wait/NOOP in the middle 
    #This causes a collision with agent 0 in the second step of the plan
    plan_1 = [move(north), move(west)]
    result_1 = p.merge_plan_into_master(0, plan_0)
    result_2 = p.merge_plan_into_master(1, plan_1)
    assert result_1 and result_2 and \
        p.get_plan_of_agent(0) == plan_0 and \
        p.get_plan_of_agent(1) == [move(north), NOOP, NOOP, move(west)] and \
        p.get_plan_of_agent(2) == []

def test_returned_plans_are_defensive_copies():
    #Make a plan and make defensive copy to be sure
    plan_0 = [move(east), move(south), move(west)]
    plan_0_defensive_copy = plan_0.copy()
    
    #Initalize master plan and merge plan into it s
    p = MasterPlan(3, initial_state)
    p.merge_plan_into_master(0, plan_0)

    #Get the plan of agent 0 and change it    
    x = p.get_plan_of_agent(0)
    x[0] = "something which would never occur a real plan"
    
    #Check that the plan in master_plan remains intact
    assert p.get_plan_of_agent(0) == plan_0_defensive_copy
    