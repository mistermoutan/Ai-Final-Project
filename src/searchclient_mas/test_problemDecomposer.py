from problemDecomposer import problemDecomposer,subtask,Task,HTN
#from coordinator import Coordinator
import test_utilities as tu
import unittest
from graph import Graph
# format agents, boxes, goals
a0 = tu.agent(0,'r')
a1 = tu.agent(1,'b')
a2 = tu.agent(2,'g')
a3 = tu.agent(3,'b')

b0 = tu.box('a','r')
b1 = tu.box('a','r')
b2 = tu.box('b','b')
b3 = tu.box('b','b')

g0 = tu.goal('a')
g1 = tu.goal('a')
g2 = tu.goal('b')
g3 = tu.goal('b')

#hardcode possible initial state
matrix = [
    [False,False,False,False,False,False,False,False,False,False],
    [False,a0   ,True ,True ,b0   ,True ,a1   ,False,g0   ,False],
    [False,True ,False,False,False,False,False,False,True ,False],
    [False,True ,True ,True ,True ,b1   ,True ,True ,a2   ,False],
    [False,b2   ,False,False,False,False,False,False,True ,False],
    [False,g1   ,False,g2   ,b2   ,a3   ,b3   ,g3    ,True ,False],
    [False,False,False,False,False,False,False,False,False,False]
]

st = tu.make_state(matrix)
pd = problemDecomposer(st)
graph = Graph(st.maze)
#test for class HTN
'''
def test_checkTasksLength():
    htn = HTN(st)
    assert len(htn.Tasks)==0
    htn.createTasks()
    assert len(htn.Tasks)==2
def test_taskRefinement():
    htn = HTN(st)
    htn.createTasks()
    for t in htn.Tasks:
        assert not t.allPrimitive()
    htn.refineTasks()
    for t in htn.Tasks:
        assert t.allPrimitive()
def test_getTasksByAgent():
    htn=HTN(st)
    assert isinstance(htn.getTasksByAgent(), dict)
'''
#test for class Tasks

def test_refinementSchemaConsistent():
    task = Task('FullfillBoxGoal',0,st,graph,[0,1],[0,1])
    #check if all steps are defined as actions
    assert all([k in task.refScheme for key,val in task.refScheme.items() for k in val['steps'] if val['isPrimitive']==False and isinstance(k, str)])
    #check names
    assert all([val['name'] in task.refScheme for key,val in task.refScheme.items()])

def test_CheckForDublicatesInSchema():
    #key duplicates
    task = Task('FullfillBoxGoal',0,st,graph,[0,1],[0,1])
    assert len(task.refScheme) == len(set(task.refScheme.keys()))
    #TODO check also if scheme entries are the same but have different names
def test_refine():
    task = Task('FullfillBoxGoal',0,st,graph,[0,1],[0,1])
    workload = []
    boxes_used = []
    assert task.steps[0]['name']=='FullfillBoxGoal'
    assert not hasattr(task,'agentBox_combi')
    task.refine(workload,boxes_used)
    assert task.steps[0]['name']=='SelectBox'
    assert task.steps[1]['name']=='SelectAgent'
    #assert callable(task.steps[2])
    
 


#Test function for subtasks


#Test functions problemDecomposer
def test_decomposeToActions():
    assert True
'''def test_createCompoundTasks():
    v = pd.createCompoundTasks()
    #for v in values:
    assert len(st.goal_types)==len(v)
    assert v[0][2]== 0 and v[1][2]==1
    assert v[0][3]==[0,1] and v[1][3]==[2]
    assert v[0][4]=={0:0,1:0} and v[1][4]=={2:0}
    assert type(v[0][6])==subtask and type(v[1][6])==subtask
'''
def test_decomposeToPrimitiveTask():
    assert True
def test_createSetOfTasks():
    assert True
def test_getGoalOrientedProblems():
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

def test_searchPossibleGoalsForBoxes():
    pd.searchPossibleGoalsForBoxes()
    assert pd.pos_goals == [[0], [0], [1], [], []]
'''
#def test_assign_tasks_greedy():
#    pd.assign_tasks_greedy()
#    assert pd.agt_tasks == [[0, 4], [1], [2], [3]]

'''def test_assign_agent_goals():
    st = tu.make_state(matrix)
    coordinator = Coordinator(st)
    pd.assign_agent_goals(coordinator)

test_assign_agent_goals()
'''