from problemDecomposer import problemDecomposer
import test_utilities as tu

# format agents, boxes, goals
a0 = tu.agent(0,'r')
a1 = tu.agent(1,'r')
a2 = tu.agent(2,'g')
a3 = tu.agent(3,'b')

b0 = tu.box('a','r')
b1 = tu.box('a','r')
b2 = tu.box('b','g')
b3 = tu.box('c','b')
b4 = tu.box('c','r')

g0 = tu.goal('a')
g1 = tu.goal('a')
g2 = tu.goal('b')
g3 = tu.goal('c')
g4 = tu.goal('c')



#hardcode possible initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,a1,True,g2,True,True,a0,False,g0,False],
    [False,True,False,False,False,False,False,False,b0,False],
    [False,True,True,g3,True,b1,True,True,g4,False],
    [False,b2,False,False,False,False,False,False,True,False],
    [False,g1,False,a2,True,True,b3,True,b4,a3],
    [False,False,False,False,False,False,False,False,False,False]
]

st = tu.make_state(matrix)
pd = problemDecomposer(st)

#Test functions
def test_createSetOfTasks():
    print(pd.createSetOfTasks())

def test_checkGoals():
    print(pd.checkGoals())

def test_searchPossibleAgentsForBoxIndex():
    assert pd.searchPossibleAgentsForBoxIndex(0) == [0,1]

def test_searchAgentsForAllBoxes():
    pd.searchAgentsForAllBoxes()
    assert pd.agts_for_boxes == [[0, 1], [0, 1], [2], [3], [0, 1]]

def test_searchPossibleBoxesForGoals():
    pd.searchPossibleBoxesForGoals()
    assert pd.pos_boxes == [[0, 1], [2]]

def test_searchPossibleGoalsForBox():
    assert pd.searchPossibleGoalsForBox()
    #assert pd.pos_goals == [[0], [0], [1], [], []]

def test_assign_tasks_greedy():
    pd.assign_tasks_greedy()
    #assert pd.agt_tasks == [[0, 4], [1], [2], [3]]

print("box_colors:   "+ str(st.box_colors))
print("box_type:     "+ str(st.box_types))
print("agent_colors: "+ str(st.agent_colors))
print("goal_type:    "+ str(st.goal_types))

#Tests
#test_searchPossibleAgentsForBoxIndex()
#test_searchAgentsForAllBoxes()
#test_searchPossibleBoxesForGoals()
#test_searchPossibleGoalsForBox()
test_assign_tasks_greedy()
