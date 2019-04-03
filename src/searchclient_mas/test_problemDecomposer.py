from problemDecomposer import problemDecomposer
import test_utilities as tu

a0 = tu.agent(0,'r')
a1 = tu.agent(1,'g')
a2 = tu.agent(2,'b')

b0 = tu.box('a','r')
b1 = tu.box('b','g')
b2 = tu.box('c','b')

g0 = tu.goal('a')
g1 = tu.goal('b')

#hardcode initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,a1,True,True,True,True,a0,False,g0,False],
    [False,True,False,False,False,False,False,False,b0,False],
    [False,True,True,True,True,b1,True,True,True,False],
    [False,b2,False,False,False,False,False,False,True,False],
    [False,g1,False,a2,True,True,True,True,True,True],
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

def test_searchPossibleAgentsForBox():
    pd.searchPossibleAgentsForBox(0)
    print(pd.agts_to_box)
    #assert agents == [0,1]

def test_searchPossibleBoxesForGoals():
    pd.searchPossibleBoxesForGoals()
    print(pd.pos_boxes)
    #assert

def test_searchPossibleGoalsForBoxes():
    pd.searchPossibleGoalsForBoxes()
    print(pd.pos_goals)
    #assert


print("boxes:  "+ str(st.box_colors))
print("agents: "+ str(st.agent_colors))

test_searchPossibleAgentsForBox()
#test_searchPossibleBoxesForGoals()
#test_searchPossibleGoalsForBoxes()
