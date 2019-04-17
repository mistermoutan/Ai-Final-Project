import test_utilities as tu
from coordinator import Coordinator
import unittest
# format agents, boxes, goals
a0 = tu.agent(0,'r')
a1 = tu.agent(1,'r')
a2 = tu.agent(2,'g')
a3 = tu.agent(3,'b')

b0 = tu.box('a','r')
b1 = tu.box('a','r')
b2 = tu.box('a','r')
b3 = tu.box('a','r')
b4 = tu.box('a','r')

g0 = tu.goal('a')
g1 = tu.goal('a')
g2 = tu.goal('a')
g3 = tu.goal('a')
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

st = tu.make_state(matrix)
st_SA = st.get_StateSA(0, True)
coordinator = Coordinator(st)

def test_distance_to():
    print(coordinator.distance_to((1,2),(1,8)) )
    #assert coordinator.distance_to((1,1),(4,4)) == 7

def test_min_distance_to_position_in_list():
    box = (1,1)
    goals = [(3,3),(4,4),(4,8)]
    assert coordinator.min_distance_to_position_in_list(box, goals) == 5

def test_distances_to_position_in_list():
    closest_boxes = [1, 0, 2, 3, 4]
    boxes = st_SA.box_positions
    agent = (st_SA.agent_row, st_SA.agent_col)
    #print(coordinator.distances_to_position_in_list(agent, [boxes[i] for i in closest_boxes]))

def test_heuristic():
    #print(coordinator.heuristic(st_SA))
    #assert coordinator.heuristic(st_SA) == 7

def test_ind_n_dis_goals_to_closest_box():
    boxes = st_SA.box_positions
    goals = st_SA.goal_positions

    #print(coordinator.ind_n_dis_goals_to_closest_box(st_SA, boxes, goals))

def test_heuristic_adv():
    #print(coordinator.heuristic_adv(st_SA))

#test_distance_to()
#test_min_distance_to_position_in_list()
#test_distances_to_position_in_list()
#test_ind_n_dis_goals_to_closest_box()

#test_heuristic()
#test_heuristic_adv()


#60
#58
#56


#74 : 16 4
