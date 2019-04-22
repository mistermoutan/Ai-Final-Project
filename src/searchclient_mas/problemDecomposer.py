from state import StateMA,StateBuilder
from typing import List, Tuple
import sys
from graph import Graph
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
class HTN():
    def __init__(self,state):
        self.state = state
        self.Tasks = []
        self.pd = problemDecomposer(state)

    def createTasks(self):
        for i in range(len(self.state.goal_types)):
            #TODO if goal type is box goal
            boxes = self.pd.searchPossibleBoxesForGoalIndex(i)
            #TODO check if goal can be achieved by agents of different color...
            agents= self.pd.searchPossibleAgentsForBox(boxes[0])
            #TODO implement precondition data structure
            self.Tasks.append(Task('FullfillBoxGoal',i,boxes,agents))
    def refineTasks(self):
        for t in self.Tasks:
            while not t.allPrimitive():
                t.refine()
    def sortTasks(self):
        self.Tasks = sorted(self.Tasks, key=lambda k: k.weight,reverse=True)
    def getTasks(self):
        return self.Tasks
    def getTasksByAgent(self):
        agentTask={}
        #TODO outsource the next three lines
        self.createTasks()
        self.refineTasks()
        self.sortTasks()

        for t in self.Tasks:
            if t.agent not in agentTask:
                agentTask[t.agent]=[t]
            else:
                agentTask[t.agent].append(t)

        return agentTask
    def createPlanByAgent(self):
        pass
    def mergePlanInMasterPlan(self):
        pass

class Task():
    def __init__(self,headTask,goal,posBoxes=[],posAgents=[]):
        #refinement schema for all actions
        #TODO make sure that headTask is in refScheme -> exception handling
        #schema = {'name':'','precond':[],'steps':[],'isPrimitive':False}
        #TODO case if precondition are met already-> do anyway?
        self.posAgents=posAgents
        self.agent=None
        self.posBoxes = posBoxes
        self.box = None
        self.refScheme = {\
            'FullfillBoxGoal': {'name':'FullfillBoxGoal','precond':[self.agentAvl,self.boxAvl,self.inSameRoom],'steps':['SelectBox','SelectAgent','MoveAgentToBox', 'MoveAgentWithBox'],'isPrimitive':False},\
            'MoveAgentToBox':{'name':'MoveAgentToBox','precond':[self.agentAvl],'steps':[],'isPrimitive':True},\
            'MoveAgentWithBox':  {'name':'MoveAgentWithBox','precond':[self.agentAdjToBox],'steps':[],'isPrimitive':True},\
            'SelectBox':{'name':'SelectBox','precond':[],'steps':[self.weightBoxes,self.selectBox],'isPrimitive':True},\
            'WeightBoxes':{'name':'WeightBoxes','precond':[],'steps':[],'isPrimitive':True},\
            'SelectAgent':{'name':'SelectAgent','precond':[],'steps':[self.weightAgents,self.selectAgent],'isPrimitive':True},\
            'WeightAgents':{'name':'WeightAgents','precond':[],'steps':[],'isPrimitive':True},\
            'MoveAgent':{'name':'MoveAgent','precond':[self.agentAvl],'steps':[],'isPrimitive':True}
        }
        self.Tasks = [self.refScheme[headTask]]
        self.weight=0
        #start refinement loop
        self.tasks = []

    #
    def refine(self):
        self.Tasks = [self.refScheme[y] for x in self.Tasks for y in (self.refScheme[x['name']]['steps'] if x['name'] in self.refScheme and x['isPrimitive']==False else [x])]
        return self
    def allPrimitive(self):
        return all([i['isPrimitive'] for i in self.Tasks])
    #effects and preconditions
    def agentAvl(self):
        return True
    def boxAvl(self):
        return True
    def agentAdjToBox(self):
        return True
    def inSameRoom(self):
        return True
    #mapping to real actions if all actions are Primitive
    def generateActions(self):
        if self.allPrimitive():
            pass
            #find actions
    def selectAgent(self):
        self.agents=self.posAgents[0]
    def weightAgents(self):
        #TODO implement
        #use a heuristic to calc a weight
        pass
    def selectBox(self):
        self.box=self.posBoxes[0]
    def weightBoxes(self):
        #TODO implement
        #use a heuristic to get a weight
        pass





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

    def assign_agent_goals(self, coordi):

        rmvd = set()
        assigned_boxes =[]
        for i in range(len(self.state.goal_types)):
            psbl_boxes = list(set(self.searchPossibleBoxesForGoalIndex(i))-rmvd)
            psbl_boxes_pos = [self.state.box_positions[j] for j in psbl_boxes]
            print("-----------------------")
            print(i)
            print(psbl_boxes_pos)
            added_dists = coordi.distances_to_position_in_list(self.state.goal_positions[i],psbl_boxes_pos)

            for k,pb in enumerate(psbl_boxes):
                #get closest agent for box
                psbl_agents = self.searchPossibleAgentsForBoxIndex(pb)
                psbl_agents_pos = [self.state.agent_positions[n] for n in psbl_agents]
                print("k")
                print(k)
                print(psbl_agents_pos)
                # add workload to box?
                dists_box_2_agents = coordi.distances_to_position_in_list(self.state.box_positions[pb],psbl_agents_pos)
                min_idx = dists_box_2_agents.index(min(dists_box_2_agents))
                closest_agent = psbl_agents[min_idx]

                added_dists[k] += dists_box_2_agents[min_idx]
                print("added_dists")
                print(added_dists)

            assigned_boxes.append(psbl_boxes[added_dists.index(min(added_dists))])
            print("assigned_boxes")
            print(assigned_boxes)
        print(assigned_boxes)


    def getTasks(self):
        return self.Tasks


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
