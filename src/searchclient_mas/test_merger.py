from action import *
from merger import merge
from state import StateMA

north = Dir.N
east  = Dir.E
south = Dir.S
west  = Dir.W

NOOP = ActionType.Wait

def move(direction):
    return Action(ActionType.Move, direction, None)

def push(agent_dir, box_dir):
    return Action(ActionType.Push, agent_dir, box_dir)

def pull(agent_dir, box_dir):
    return Action(ActionType.Pull, agent_dir, box_dir)

maze = [
        [False,False,False,False,False],
        [False,True,True,True,False],
        [False,True,True,True,False],
        [False,True,True,True,False],
        [False,False,False,False,False]
    ]

boxes = [(3,(2,3),0)]
agent = [((2,1),1), ((3,2),0)]
goals = []

noop_action_vector = [NOOP,NOOP]
empty_master_plan = [[NOOP, NOOP]]

initial_state = StateMA(maze,boxes,goals,agent)

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

def test_can_merge_when_conflict_can_be_resolved():
    master_plan = [noop_action_vector, noop_action_vector, [NOOP, move(north)],[NOOP, move(north)]]
    agent_plan = [move(east), move(south)]
    revised_plan = merge(0, agent_plan, master_plan, 0, initial_state)
    assert revised_plan == [NOOP, NOOP, move(east), move(south)]

def test_can_merge_when_conflict_can_be_resolved_2():
    master_plan = [noop_action_vector, noop_action_vector, [move(east), NOOP], noop_action_vector, [push(north,north),NOOP]]
    agent_plan = [move(east), move(south), move(east)]
    revised_plan = merge(0, agent_plan, master_plan, 0, initial_state)
    assert revised_plan == [move(east), NOOP, move(south), NOOP, move(east)]