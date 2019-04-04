from merger import merge
from state import StateMA
import test_utilities as util
from action import move,push,pull,north,west,south,east,NOOP



#Make the level
agt0 = util.agent(0,"red")
agt1 = util.agent(1,"blue")
box0  = util.box("A", "blue")
level = [
        [False,False,False,False,False],
        [False,True,True,True,False],
        [False,agt0,True,box0,False],
        [False,True,agt1,True,False],
        [False,False,False,False,False]
    ]
initial_state = util.make_state(level)


noop_action_vector = [NOOP,NOOP]
empty_master_plan = [[NOOP, NOOP]]

def test_can_merge_into_empty_plan():
    agent_plan = [move(east)]
    revised_plan = merge(0, agent_plan, empty_master_plan, 0, initial_state)
    assert revised_plan == [move(east)]

def test_can_merge_into_empty_plan_at_random_index():
    agent_plan = [move(east)]
    revised_plan = merge(0, agent_plan, empty_master_plan, 5, initial_state)
    assert revised_plan == [move(east)]

def test_result_is_none_if_plans_not_mergable_becaue_agent_goes_into_wall():
    #Agent runs into wall
    agent_plan = [move(west)]
    revised_plan = merge(0, agent_plan, empty_master_plan, 0, initial_state)
    assert revised_plan == None

def test_result_is_none_if_plans_are_not_mergable_because_agent_goes_into_agent():
    master_plan = [[NOOP, move(north)]]
    agent_plan = [move(east), move(south)]
    revised_plan = merge(0, agent_plan, master_plan, 0, initial_state)
    assert revised_plan == None

def test_result_is_none_if_plans_are_not_mergable_because_agent_goes_into_box_with_different_color():
    agent_plan = [move(east), move(east)]
    revised_plan = merge(0, agent_plan, empty_master_plan, 0, initial_state)
    assert revised_plan == None

def test_can_merge_when_conflict_can_be_resolved_1():
    master_plan = [noop_action_vector, noop_action_vector, [NOOP, move(north)],[NOOP, move(north)]]
    agent_plan = [move(east), move(south)]
    revised_plan = merge(0, agent_plan, master_plan, 0, initial_state)
    assert revised_plan == [NOOP, NOOP, NOOP, move(east), move(south)]

def test_can_merge_when_conflict_can_be_resolved_2():
    master_plan = [noop_action_vector, noop_action_vector, [NOOP, move(east)], noop_action_vector, [NOOP,push(north,north)]]
    agent_plan = [move(east), move(south), move(east)]
    revised_plan = merge(0, agent_plan, master_plan, 0, initial_state)
    assert revised_plan == [move(east), NOOP, move(south), NOOP, move(east)]

def test_can_merge_when_conflict_can_be_resolved_3():
    master_plan = [noop_action_vector, noop_action_vector, [move(east), NOOP], [move(north), NOOP], noop_action_vector, [move(east), NOOP]]
    agent_plan = [move(west), move(north), move(north), move(east)]
    revised_plan = merge(1, agent_plan, master_plan, 0, initial_state)
    assert revised_plan == [move(west), NOOP, move(north), move(north), NOOP, move(east)] or \
           revised_plan == [move(west), NOOP, move(north), NOOP, move(north), move(east)]

"""
This is slow. Takes about 20 seconds. Don't think it'll be fast enough
def test_can_merge_for_large_plans():
    master_plan = [noop_action_vector]*1000
    master_plan.append([NOOP, move(east)])
    agent_plan = [move(south), move(east)]
    revised_plan = merge(0, agent_plan, master_plan, 0, initial_state)
    print("done")
"""
