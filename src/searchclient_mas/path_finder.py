from action import Action,move,push,pull,north,south,east,west
from graph_alternative import Graph
from typing import Tuple,List
from utilities import pairwise,next_position_in_direction,are_adjacent,direction_between_adjacent_positions

def path_between_positions_as_directions(graph, source, target):
    path = graph.shortest_path_between(source,target)
    return [direction_between_adjacent_positions(p1,p2) for (p1,p2) in pairwise(path)]

def actions_to_move_between(graph : Graph, source: Tuple[int,int], target: Tuple[int,int]) -> List[Action]:
    path = path_between_positions_as_directions(graph,source,target)
    return [move(direction) for direction in path]

def actions_to_push_box_between(graph, agent, box, target):
    
    assert are_adjacent(agent,box), "Agent and box should be adjacent"

    #Case where box is already at target
    if box == target:
        return []

    #Shortest path from box to target in terms of the directions on that path
    path_box_to_target = path_between_positions_as_directions(graph,box, target)
    
    #Check that agent does not block the box's path to target
    first_direction_of_box = path_box_to_target[0]
    next_position_on_path = next_position_in_direction(box, first_direction_of_box)
    assert not agent == next_position_on_path, "agent is on shortest path from box to target. Cannot be pushed"

    #The first push action needs the agents location in relation to the box
    agent_to_box_dir = direction_between_adjacent_positions(agent,box)
    push_path = [(agent_to_box_dir,first_direction_of_box)]
    #Add every pair of consecutive directions from the shortest path to the push path
    push_path.extend( pairwise(path_box_to_target) )
    #Convert every pair of directions to a push action
    return [push(d1,d2) for (d1,d2) in push_path]

#Moving the agent to a target position while pulling a box is equivalent to swapping the location of the agent and 
#the box and having the agent push the box to the same location.
#Therefore we use actions_to_push_box_between and convert each push action to the corresponding pull action
def actions_to_move_to_target_while_pulling_box(graph, agent, box, target):
    box_push_agent_path = actions_to_push_box_between(graph, box, agent, target)
    convert = lambda push_action: pull(push_action.box_dir, push_action.agent_dir)
    return [convert(push_action) for push_action in box_push_agent_path]

def first_off_path_node(graph, path):
    assert len(path) > 0, "paths of length zero have no off path nodes"
    nodes_in_path = set(path)
    
    #Traverse that path from start to finish and return one of the first neighbours found that are 
    #not on the path themselves
    for node in path:
        neighbours = [v for v in graph.get_neighbours(node) if not v in nodes_in_path]
        if not neighbours:
            continue
        return neighbours[0]

    return None

