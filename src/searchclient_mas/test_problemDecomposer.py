from problemDecomposer import problemDecomposer
import test_utilities as tu

a0 = tu.agent(0,'r')
a1 = tu.agent(1,'r')

b0 = tu.box('a','r')
b1 = tu.box('b','r')

g0 = tu.goal('a')
g1 = tu.goal('b')

#hardcode initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,True,True,True,True,True,True,a0,False,False],
    [False,True,False,False,False,False,False,False,g0,False],
    [False,True,True,True,True,True,True,True,True,False],
    [False,g1,False,False,False,False,False,False,True,False],
    [False,True,False,a1,True,True,True,True,True,True],
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
    print(pd.checkGoals)

def test_getPossibleAgents():
    print(pd.PossibleAgents)

def test_getTasks():
    print(pd.getTasks)


test_createSetOfTasks()
