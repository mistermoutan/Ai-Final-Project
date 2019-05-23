import test_utilities as tu
from coordinator import Coordinator
import unittest
# format agents, boxes, goals
a0 = tu.agent(0,'r')
a1 = tu.agent(1,'g')
a2 = tu.agent(2,'y')
a3 = tu.agent(3,'b')

b0 = tu.box('a','r')
b1 = tu.box('b','g')
b2 = tu.box('c','y')
b3 = tu.box('d','b')
b4 = tu.box('a','r')

g0 = tu.goal('a')
g1 = tu.goal('b')
g2 = tu.goal('c')
g3 = tu.goal('d')
g4 = tu.goal('a')



#hardcode possible initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,True ,a0   ,b0   ,True ,b1   ,True ,True ,g0   ,False],
    [False,True ,True ,True ,True ,True ,True ,True ,g1   ,False],
    [False,True ,b2   ,True ,True ,True ,True ,True ,g2   ,False],
    [False,True ,b3   ,True ,True ,True ,True ,True ,g3   ,False],
    [False,True ,True ,b4   ,True ,True ,True ,True ,g4   ,False],
    [False,False,False,False,False,False,False,False,False,False]
]

matrix2 = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,True ,True ,True ,b1   ,a0   ,True  ,False,True ,False],
    [False,True ,b3   ,b2   ,True ,True ,b0  ,True ,True ,False ],
    [False,False,False,False,False,False,False,False,False,False]
]

matrix3 = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,b3   ,False,b0   ,a0   ,False,True ,True ,True ,False],
    [False,True ,False,True ,True ,False,True ,b2   ,a1   ,False],
    [False,b2   ,False,True ,b1   ,False,True ,True ,True ,False],
    [False,True ,False,True ,b1   ,False,True ,True ,True ,False],
    [False,a2   ,False,True ,True ,False,True ,a3   ,True ,False],
    [False,False,False,False,False,False,False,False,False,False]
]

st = tu.make_state(matrix2)
st_SA = st.get_StateSA(0, True)
coordinator = Coordinator(st)

def test_distance_to():
    #shortest_path_between is currently of by 2 (or 1)
    #print(coordinator.distance_to((1,2),(1,8)) )
    bx = st.box_positions[0]
    print(bx)
    ag = st.agent_positions[0]
    print(ag)
    gl = st.goal_positions[0]
    print(gl)

    print(coordinator.distance_to((1,4),(bx)))

def test_min_distance_to_position_in_list():
    box = (1,1)
    goals = [(3,3),(4,4),(4,8)]
    assert coordinator.min_distance_to_position_in_list(box, goals) == 5

def test_distances_to_position_in_list():
    closest_boxes = [1, 0, 2, 3, 4]
    boxes = st_SA.box_positions
    agent = (st_SA.agent_row, st_SA.agent_col)
    assert coordinator.distances_to_position_in_list(agent, [boxes[i] for i in closest_boxes]) == [4, 2, 3, 4, 6]


def test_ind_n_dis_goals_to_closest_box():
    boxes = st_SA.box_positions
    goals = st_SA.goal_positions
    #print(coordinator.ind_n_dis_goals_to_closest_box(st_SA, boxes, goals))
    #assert coordinator.ind_n_dis_goals_to_closest_box(st_SA, boxes, goals) == ([1, 0, 2, 3, 4], [3, 6, 6, 6, 5])

def test_heuristic():
    assert coordinator.heuristic(st_SA) == 8

def test_heuristic_adv():
    st_SA.goal_types = ['a', 'b', 'c', 'd']
    st_SA.goal_by_cords = {(2, 5): 0, (2, 6): 1, (2, 7): 2, (2, 8): 3}
    st_SA.goal_positions = [(2, 5), (2, 6), (2, 7), (2, 8)]
    print(coordinator.heuristic_adv(st_SA))
    #assert coordinator.heuristic_adv(st_SA) == 142

def test_remove_unused_boxes():
    coordinator.get_immovable_boxes()

#test_distance_to()
#test_min_distance_to_position_in_list()
#test_distances_to_position_in_list()
#test_ind_n_dis_goals_to_closest_box()

#test_heuristic()
#test_heuristic_adv()
test_remove_unused_boxes()
