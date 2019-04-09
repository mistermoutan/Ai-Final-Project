from problemDecomposer import problemDecomposer,subtask,HTN
import test_utilities as tu
import unittest
# format agents, boxes, goals
a0 = tu.agent(0,'r')
a1 = tu.agent(1,'r')
a2 = tu.agent(2,'g')
a3 = tu.agent(3,'b')

b0 = tu.box('a','r')
b1 = tu.box('a','r')
b2 = tu.box('b','g')
b3 = tu.box('c','b')

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
    [False,g1,False,a2,True,True,b3,True,False,a3],
    [False,False,False,False,False,False,False,False,False,False]
]

st = tu.make_state(matrix)
pd = problemDecomposer(st)

#Test functions for HTN

def test_selectAgent():
    assert True
def test_selectBox():
    assert True
def test_planTask():
    assert True
def test_solve():
    assert True

#Test function for subtasks
def test_MoveAgent():
    st = subtask()
    assert st.child==None
    assert st.task==None
    st.MoveAgent((1,2),(2,2))
    assert st.task==0
    assert st.fromPos[0] == 1 and st.fromPos[1] == 2
    assert st.toPos[0]   == 2 and st.toPos[1]   == 2
def test_MoveAgentWithBox():
    st = subtask()
    assert st.child==None
    assert st.task==None
    st.MoveAgentWithBox((1,2),(2,2))
    assert st.task==1
    assert st.fromPos[0] == 1 and st.fromPos[1] == 2
    assert st.toPos[0]   == 2 and st.toPos[1]   == 2
def test_addChilTask():
    st = subtask()
    assert type(st) is subtask
    assert st.child == None
    st.addChildTask()
    assert type(st.child) is subtask
    assert st.child.child == None
    st.child.addChildTask()
    assert type(st.child.child) is subtask
def test_getChildTask():
    assert True

#Test functions problemDecomposer
def test_decomposeToActions():
    assert True
def test_createCompoundTasks():
    v = pd.createCompoundTasks()
    #for v in values:
    assert len(st.goal_types)==len(v)
    assert v[0][2]== 0 and v[1][2]==1
    assert v[0][3]==[0,1] and v[1][3]==[2]
    assert v[0][4]=={0:0,1:0} and v[1][4]=={2:0}
    assert type(v[0][6])==subtask and type(v[1][6])==subtask

def test_decomposeToPrimitiveTask():
    assert True
def test_createSetOfTasks():
    assert True
def test_getGoalOrientedProblems():
    assert True
def test_sortbyWeight():
    x = {1: 2, 3: 4, 4: 3, 2: 1, 0: 0}
    test = pd.sortbyWeight(x)
    assert True
def test_searchPossibleAgentsForBox():
    assert True
def test_searchPossibleBoxesForGoalIndex():
    assert True
def test_searchPossibleGoalsForBoxIndex():
    assert True
def test_searchPossibleAgentsForBoxIndex():
    assert True
def test_getTasks():
    assert True


'''
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

test_createCompoundTasks()
'''
