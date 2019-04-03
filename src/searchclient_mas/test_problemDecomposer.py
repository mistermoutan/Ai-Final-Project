from problemDecomposer import problemDecomposer
import test_utilities as tu

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

#hardcode initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,a1,True,True,True,True,a0,False,g0,False],
    [False,True,False,False,False,False,False,False,b0,False],
    [False,True,True,True,True,b1,True,True,True,False],
    [False,b2,False,False,False,False,False,False,True,False],
    [False,g1,False,a2,True,True,b3,True,b4,a3],
    [False,False,False,False,False,False,False,False,False,False]
]
# initial StateMA
# ++++++++++++
# +       0+ +
# + ++++++++A+
# +          +
# +B++++++++ +
# + +1       +
# ++++++++++++

st = tu.make_state(matrix)
pd = problemDecomposer(st)

#Test functions
def test_createSetOfTasks():
    print(pd.createSetOfTasks())

def test_checkGoals():
    print(pd.checkGoals())

def test_searchPossibleAgentsForBoxIndex():
    print(pd.searchPossibleAgentsForBoxIndex(0))
    #assert agents == [0,1]

def test_searchAgentsForAllBoxes():
    pd.searchAgentsForAllBoxes()
    print(pd.agts_for_boxes)
    #assert agents == [0,1]

def test_searchPossibleBoxesForGoals():
    pd.searchPossibleBoxesForGoals()
    print(pd.pos_boxes)
    #assert

def test_searchPossibleGoalsForBoxes():
    pd.searchPossibleGoalsForBoxes()
    print(pd.pos_goals)
    #assert

def test_assign_tasks_greedy():
    pd.assign_tasks_greedy()
    print(pd.agt_tasks)

print("boxes:  "+ str(st.box_colors))
print("agents: "+ str(st.agent_colors))

#test_searchPossibleAgentsForBoxIndex()
#test_searchAgentsForAllBoxes()
#test_searchPossibleBoxesForGoals()
#test_searchPossibleGoalsForBoxes()
test_assign_tasks_greedy()
