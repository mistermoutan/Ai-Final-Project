from graph_alternative import Graph
from path_finder import actions_to_move_between,actions_to_push_box_between,actions_to_move_to_target_while_pulling_box,first_off_path_node
from action import move,push,pull,north,south,east,west
import pytest

level = [
     [False,False,False,False,False],
     [False,True ,False,True ,False],
     [False,True ,False,True ,False],
     [False,True ,True ,True ,False],
     [False,False,True ,False,False],
     [False,True ,True ,False,False],
     [False,False,False,False,False]
]

graph = Graph(level)

def test_path_to_move_between_same_vertex():
    source = (1,1)
    target = (1,1)
    path = actions_to_move_between(graph,source,target)
    assert path == []


def test_path_to_move_between_adjacent_vertices_1():
    source = (1,1)
    target = (2,1)
    path = actions_to_move_between(graph, source,target)
    assert path == [move(south)]

#Same as above, just reversed source and target
def test_path_to_move_between_adjacent_vertices_2():
    source = (2,1)
    target = (1,1)
    path = actions_to_move_between(graph, source,target)
    assert path == [move(north)]

def test_path_to_move_between_distant_vertices():
    source = (1,3)
    target = (5,1)
    path = actions_to_move_between(graph, source, target)
    print("actual", path)
    expected = [move(south), move(south), move(west), \
                move(south), move(south), move(west)]
    print("expected", expected)
    assert path == expected

#test_path_to_move_between_distant_vertices()

def test_actions_to_push_box_raise_error_if_agent_not_next_to_box():
    with pytest.raises(Exception):
        agent_source = (1,1)
        box_source   = (1,3)
        box_target   = (5,1)
        actions_to_push_box_between(graph, agent_source, box_source, box_target)

def test_actions_to_push_box_raise_error_if_agent_and_box_at_same_vertex():
    with pytest.raises(Exception):
        agent_and_box_source = (1,1)
        target = (1,3)
        actions_to_push_box_between(graph, agent_and_box_source, agent_and_box_source, target)

#Consider the shortest path from a box to the target you want to push it to. 
#If the agent is adjacent to the box, but on this path, it isn't possible to
#only push the box along the shortest path to get it to the target. At least 
#one pull operation is necessary to move the box around the agent
#Of course, it might be possible to only push the box if there is some cycle in the level
#But this cycle is not on the shortest path
def test_actions_to_push_box_raise_error_if_agent_on_path_from_box_to_target():
    with pytest.raises(Exception):
        agent  = (2,1)
        box    = (1,1)
        target = (1,3)
        actions_to_push_box_between(graph, agent, box, target)

def test_actions_to_push_box_empty_if_box_at_target():
    agent  = (1,1)
    box    = (2,1)
    target = (2,1)
    path  = actions_to_push_box_between(graph, agent, box, target)
    assert path == []

def test_actions_to_push_box_one_step_straight_path():
    agent  = (1,1)
    box    = (2,1)
    target = (3,1)
    path  = actions_to_push_box_between(graph, agent, box, target)
    assert path == [push(south,south)]

def test_actions_to_push_box_one_step_around_corner():
    agent  = (2,1)
    box    = (3,1)
    target = (3,2)
    path  = actions_to_push_box_between(graph, agent, box, target)
    assert path == [push(south,east)]

def test_actions_to_push_box_distant_positions_1():
    agent  = (1,1)
    box    = (2,1)
    target = (5,1)
    path = actions_to_push_box_between(graph, agent, box, target)
    assert path == [push(south, south), push(south,east), push(east,south), push(south,south), push(south,west)]

def test_actions_to_push_box_distant_positions_2():
    agent  = (1,1)
    box    = (2,1)
    target = (1,3)
    path = actions_to_push_box_between(graph, agent, box, target)
    assert path == [push(south, south), push(south,east), push(east,east), push(east,north), push(north,north)]

def test_actions_to_move_and_pull_box_raises_error_if_agent_not_next_to_box():
    with pytest.raises(Exception):
        agent =  (1,1)
        box   =  (1,3)
        target = (4,2)
        actions_to_move_to_target_while_pulling_box(graph, agent, box, target)

def test_actions_to_move_and_pull_box_raises_error_if_agent_and_box_on_same_location():
    with pytest.raises(Exception):
        agent =  (1,1)
        box   =  (1,1)
        target = (4,2)
        actions_to_move_to_target_while_pulling_box(graph, agent, box, target)

def test_actions_to_move_and_pull_box_raises_error_if_box_is_on_shortest_path_from_agent_to_target():
    with pytest.raises(Exception):
        agent =  (1,1)
        box   =  (2,1)
        target = (4,2)
        actions_to_move_to_target_while_pulling_box(graph, agent, box, target)

def test_actions_to_move_and_pull_box_is_empty_if_agent_already_on_target():
    agent  = (1,1)
    box    = (2,1)
    target = (1,1)
    path = actions_to_move_to_target_while_pulling_box(graph, agent, box, target)
    assert path == []

def test_actions_to_move_and_pull_box_one_step_straight_path():
    agent  = (2,1)
    box    = (1,1)
    target = (3,1)
    path  = actions_to_move_to_target_while_pulling_box(graph, agent, box, target)
    assert path == [pull(south,south)]

def test_actions_to_move_and_pull_box_one_step_around_corner():
    agent  = (3,1)
    box    = (2,1)
    target = (3,2)
    path  = actions_to_move_to_target_while_pulling_box(graph, agent, box, target)
    assert path == [pull(east,south)]

def test_actions_to_move_and_pull_box_distant_positions_1():
    agent  = (2,1)
    box    = (1,1)
    target = (5,1)
    path = actions_to_move_to_target_while_pulling_box(graph, agent, box, target)
    assert path == [pull(south,south), pull(east,south),pull(south,east), pull(south,south), pull(west,south)]

def test_actions_to_move_and_pull_box_distant_positions_2():
    agent  = (2,1)
    box    = (1,1)
    target = (1,3)
    path = actions_to_move_to_target_while_pulling_box(graph, agent, box, target)
    assert path == [pull(south,south), pull(east,south), pull(east,east), pull(north,east), pull(north,north)]

#Test function to find off path node
def test_find_first_off_path_node_empty_oath_throws_error():
        with pytest.raises(Exception):
                first_off_path_node(graph, [])

def test_find_first_off_path_node_for_single_node_without_neighbours_throws_error():
        #a level with only one vertex
        graph = Graph([[True]])
        with pytest.raises(Exception):
                first_off_path_node(graph, [(0,0)])

def test_find_first_off_path_node_one_element_returns_only_neighbour():
        assert first_off_path_node(graph, [(1,1)]) == (2,1)
        assert first_off_path_node(graph, [(1,3)]) == (2,3)
        assert first_off_path_node(graph, [(5,1)]) == (5,2)

def test_find_first_off_path_one_element_returns_one_neighbour():
        assert first_off_path_node(graph, [(2,1)]) in [(1,1),(3,1)]
        assert first_off_path_node(graph, [(4,2)]) in [(3,2),(5,2)]

def test_find_first_off_path_node_long_path():
        #Path from (1,1) to (3,1)
        path = [(1,1),(2,1),(3,1),(3,2),(3,3),(3,2),(3,1)]
        assert first_off_path_node(graph, path) == (4,2)

def test_find_first_off_path_node_long_path_multiple_possibilities():
        #Path from (3,2) to (1,3)
        path = [(3,2),(3,3),(2,3),(1,3)]
        assert first_off_path_node(graph, path) in [(3,1),(4,2)]