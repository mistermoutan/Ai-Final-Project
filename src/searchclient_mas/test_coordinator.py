import test_utilities as tu
from coordinator import Coordinator
import unittest
# format agents, boxes, goals
a0 = tu.agent(0,'r')
a1 = tu.agent(1,'r')
a2 = tu.agent(2,'g')
a3 = tu.agent(3,'b')

b0 = tu.box('a','r')
b1 = tu.box('b','r')
b2 = tu.box('c','r')
b3 = tu.box('c','b')

g0 = tu.goal('a')
g1 = tu.goal('b')
g2 = tu.goal('c')
g3 = tu.goal('c')
g4 = tu.goal('c')



#hardcode possible initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,True ,True ,True ,True ,True ,True ,True ,True ,False],
    [False,True ,True ,True ,True ,True ,True ,True ,True ,False],
    [False,True ,True ,True ,True ,True ,g2   ,True ,True ,False],
    [False,b1   ,True ,True ,True ,True ,a0   ,True ,True ,False],
    [False,g1   ,True ,True ,b0   ,True ,g0   ,True ,b2   ,False],
    [False,False,False,False,False,False,False,False,False,False]
]

st = tu.make_state(matrix)
st_SA = st.get_StateSA(0, True)
coordinator = Coordinator(st)

def test_heuristic_2():
    print(coordinator.heuristic_2(st_SA))


test_heuristic_2()
