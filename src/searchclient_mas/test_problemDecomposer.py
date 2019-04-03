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
g1 = tu.goal('b')

#hardcode possible initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,a1,True,True,True,True,a0,False,g0,False],
    [False,True,False,False,False,False,False,False,b0,False],
    [False,True,True,True,True,b1,True,True,True,False],
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

def test_searchPossibleGoalsForBoxes():
    pd.searchPossibleGoalsForBoxes()
    assert pd.pos_goals == [[0], [0], [1], [], []]

def test_assign_tasks_greedy():
    pd.assign_tasks_greedy()
    assert pd.agt_tasks == [[0, 4], [1], [2], [3]]

#print("boxes:  "+ str(st.box_colors))
#print("agents: "+ str(st.agent_colors))

#Tests
test_searchPossibleAgentsForBoxIndex()
test_searchAgentsForAllBoxes()
test_searchPossibleBoxesForGoals()
test_searchPossibleGoalsForBoxes()
test_assign_tasks_greedy()
