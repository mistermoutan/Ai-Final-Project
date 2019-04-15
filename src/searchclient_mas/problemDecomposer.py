from state import StateMA,StateBuilder
from typing import List, Tuple
import sys
from graph_alternative import Graph
from path_finder import actions_to_move_between,actions_to_push_box_between,actions_to_move_to_target_while_pulling_box,first_off_path_node
from action import move,push,pull,north,south,east,west
'''
idea
    - get list of tasks/problems
    - decompose to primitive tasks
    - use planner plan pathes for tasks
    - weight tasks
    - sort tasks by weighting
    - decompose tasks to actions
    - send actions
'''
'''

refine until it is primitive
'''
class Task():
    
    def __init__(self):
        self.dispatcher = {\
        'TransportBox': {'func':[self.MoveAgent(), self.MoveAgentWithBox()],'precond':[],'effec':[],'isPrimitive':False},\
        'MoveAgent':  {'func':[actions_to_move_between],'precond':[],'effec':[self.agentAdjToBox],'isPrimitive':True},\
        'MoveAgentWithBox':  {'func':[actions_to_push_box_between],'precond':[self.agentAdjToBox],'effec':[],'isPrimitive':True},\
        }
        self.isPrimitive=False
        self.weight=0
        #start refinement loop
        self.primActions = []

    #
    def refine(self):
        pass
    #refinement functions
    def TransportBox(self):
        pass
    def MoveAgent(self):
        pass
    def MoveAgentWithBox(self):
        pass

    
    #checking preconditions

    #effects
    def agentAdjToBox(self):
        return True        





        
class CompoundTask():
    def __init__(self,isPrimitive,name,goadId,boxes=[],agents=[],precond=None):
        self.isPrimitive = isPrimitive
        self.name = name
        self.goadId = goadId
        self.boxes = boxes
        self.agents = agents
        self.precond = precond
        self.subtask = subtask()
    def WeightAgent(self):
        pass
    def WeightBoxes(self):
        pass
    def getSubtasks(self):
        return self.subtask
    def getAllPrimitiveActions(self):
        pass

class subtask():
    def __init__(self):
        self.child=None
        self.task = None
        self.fromPos = None
        self.toPos = None
        self.primitiveTasks=None
        self.isPrimitive=False
        self.actions = []
        self.haspParent=False
    def ClearPath(self):
        pass
    def MoveAgent(self,fromPos=(0,0),toPos=(0,0),graph=None):
        self.task = 1
        self.fromPos=fromPos
        self.toPos = toPos
        self.isPrimitive=True
        #self.actions = actions_to_move_between(graph,fromPos,toPos)
    def MoveAgentWithBox(self,fromPos=(0,0),toPos=(0,0)):
        self.task = 2
        self.fromPos=fromPos
        self.toPos = toPos
        self.isPrimitive=True
        #decide between between push or pull
        self.actions = None #actions_to_push_box_between(graph,)
    def addChildTask(self):
        self.child=subtask()
        self.child.haspParent=True
    def getChildTask(self):
        return self.child
'''
This class decomposes a level into high level problems where
'''
class problemDecomposer():
    def __init__(self,state):
        self.Tasks  = []
        self.Plan = []
        self.state = state
        #print(self.state.box_types,file= sys.stderr, flush=True)
    def createCompoundTasks(self):
        #create one compound task for each goal
        '''
        (isPrimitive(bool),name(str),to(int),from(List(int)),with(List(int),precond[],effect[]'
        (False,'TransportTo',goal_index,[1,2,...(box_index)],precond[x1[],x2[]],subtask:[x1][x2]]
        if subtask empty => isPrimitive = True
        '''
        g = []
        for i in range(len(self.state.goal_types)):
            boxes = self.searchPossibleBoxesForGoalIndex(i)
            #TODO check if goal can be achieved by agents of different color...
            agents= self.searchPossibleAgentsForBox(boxes[0])
            #TODO implement precondition data structure
            precond=[]
            g.append(CompoundTask(False,'TransportTo',i,boxes,agents,precond))
            #g.append((False,'TransportTo', i,boxes,agents,precond,subtasks))
        self.Plan=g
        return g

    def decomposeToActions(self,task):
        pass
    def decomposeToPrimitiveTask(self,task):
        #(False, 'TransportTo', 0, [0, 1], {0: 0, 1: 0}, [], subtask()
        if not task[0]:
            if task[1]=='TransportTo':
                #TODO: pick agent with by weight
                agent = task[4]
                #if not at box
                #task[6].MoveAgent().MoveAgentWithBox().MoveAgent



    def sortbyWeight(self,obj):
        return sorted(obj.items(), key=lambda x: x[1])

        '''
        Checks if all boxes are at their goal state
        returns list of boolean where true represents a need for a task
        '''
    def getGoalOrientedProblems(self):
        g = []
        for i in range(len(self.state.goal_types)):
            g.append((i,self.searchPossibleBoxesForGoalIndex(i)))
        self.Plan.insert(g)
        return self.Plan
    def searchPossibleAgentsForBox(self,box_idx):
        '''returns a dict of agent indexes that are able to move the box at pos box_idx in the box list and adds an initial value for the weight of an agent'''
        return {idx:0 for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]}

    def searchPossibleBoxesForGoalIndex(self,goal_idx):
        '''returns a list of box-indexes that are able to satisfy the goal at pos goad_idx in the goal list'''
        return [idx for idx in range(len(self.state.box_types)) if self.state.goal_types[goal_idx] ==self.state.box_types[idx]]

    def searchPossibleGoalsForBoxIndex(self,box_idx):
        '''returns a list of goal-indexes that are able to store the box at index box_idx'''
        return [idx for idx in range(len(self.state.goal_types)) if self.state.goal_types[idx] ==self.state.box_types[box_idx]]

    def searchPossibleAgentsForBoxIndex(self,box_idx):
        '''returns a list of agent indexes that are able to move the boxes at pos i in the box list'''
        return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]]


    def assign_tasks_greedy(self):
        '''- greedely assings tasks to agents with lowest workload that can fullfill the task
           - every box represents a task
           - returns list of lists of tasks for agents at position i
        '''
        self.agt_boxes = [[] for i in range(len(self.state.agent_colors))]

        for i,b in enumerate(self.state.box_colors):
            p_agts = self.searchPossibleAgentsForBoxIndex(i)
            if p_agts:
                ls = [len(self.agt_boxes[p]) for p in p_agts]
                if ls:
                    m = ls.index(min(ls))
                    self.agt_boxes[p_agts[m]].append(i)

                else:
                    self.agt_boxes[p_agts[-1]].append(i)

        self.agt_goals = [[] for i in range(len(self.state.agent_colors))]

        seen = []

        for i,bxs in enumerate(self.agt_boxes):
            for b in bxs:
                for goal in self.searchPossibleGoalsForBoxIndex(b):
                    if goal not in seen:
                        self.agt_goals[i].append(goal)
                        seen.append(goal)
                        break

        self.agt_tasks=[(self.agt_boxes[i],self.agt_goals[i]) for i in range(0,len(self.agt_boxes))]

        return self.agt_tasks

        #extension:
        # - pick agent that has lowest workload and is closest to box
        # - estimate workload of agents based on distance box to goal

    def getTasks(self):
        return self.Tasks
