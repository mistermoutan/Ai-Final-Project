from state import StateMA,StateBuilder
from typing import List, Tuple
import sys
from graph import Graph
from path_finder import actions_to_move_between,actions_to_push_box_between,actions_to_move_to_target_while_pulling_box,first_off_path_node
from action import move,push,pull,north,south,east,west
from master_plan import MasterPlan
from planner import Planner


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
        self.graph_of_level = Graph(state.maze)
    def createTasks(self):
        for i in range(len(self.state.goal_types)):
            print(i,file=sys.stderr,flush=True)
            #TODO if goal type is box goal 
            boxes = self.pd.searchPossibleBoxesForGoalIndex(i)
            #TODO check if goal can be achieved by agents of different color...
            agents= self.pd.searchPossibleAgentsForBoxIndex(boxes[0])
            #TODO implement precondition data structure
            self.Tasks.append(Task('FullfillBoxGoal',i,boxes,agents))
    def refineTasks(self):
        i = 0
        for t in self.Tasks:
            while not t.allPrimitive():
                t.refine()
                print('while loop '+str(i),file=sys.stderr,flush=True)
                i+=1
    def sortTasks(self):
        self.Tasks = sorted(self.Tasks, key=lambda k: k.weight,reverse=True) 
    def getTasks(self):
        return self.Tasks
    def getTasksByAgent(self):
        agentTask={}

        for t in self.Tasks:
            if t.agent not in agentTask:
                agentTask[t.agent]=[(t.goal,t.box)]
            else:
                agentTask[t.agent].append((t.goal,t.box))
        #Workaround because the agent order matters
        return {key: agentTask[key]for key in sorted(agentTask.keys())}
        
    def createPlanByAgent(self):
        pass
    def mergePlanInMasterPlan(self):
        pass
    def distance_to(self, x, y):
        return len(self.graph_of_level.shortest_path_between(x,y))

    def min_distance_to_position_in_list(self, box, goals):
        distances = [self.distance_to(box, goal) for goal in goals]
        return min(distances)
    #Simple heuristic which minimizes the value (agent_to_box + box_to_goal) distance
    def heuristic(self, state):
        agent = (state.agent_row, state.agent_col)
        boxes = state.box_positions
        goals = state.goal_positions

        agent_to_box_distances = [self.distance_to(agent, box) for box in boxes]
        box_to_goals_distances = [self.min_distance_to_position_in_list(box, goals) for box in boxes]
        agent_to_box_goal_goal_distances = [agent_to_box_distances[i] + box_to_goals_distances[i] for i in range(len(agent_to_box_distances))]
        return min(agent_to_box_goal_goal_distances)


    def make_single_agent_plan(self, initial_state):
        return Planner(initial_state, heuristic=self.heuristic, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
    
    def solve(self):
        number_of_agents = len(self.state.agent_positions)
        self.createTasks()
        self.refineTasks()
        self.sortTasks()
        single_agent_tasks = self.getTasksByAgent()
        print('sat'+str(single_agent_tasks),file=sys.stderr,flush=True)
        #run all low level functions to create primitive tasks
        single_agent_states = [self.state.get_HTN_StateSA(agent,tasks,True) for agent,tasks in single_agent_tasks.items()]

        #create single agent plans
        single_agent_plans = [self.make_single_agent_plan(s) for s in single_agent_states]

        #create masterplan
        master_plan = MasterPlan(number_of_agents, self.state)
        #merge singleAgentPlans into MasterPlan
        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)

        return master_plan.plan


class Task():
    #TODO Choose between agents based on current Workload and Distance
    #TODO Availablity of agents - init selection function in Task but run it in HTN if all Tasks are created
    #TODO Choose between boxes 
    def __init__(self,headTask,goal,posBoxes=[],posAgents=[]):
        #refinement schema for all actions
        #TODO make sure that headTask is in refScheme -> exception handling
        #schema = {'name':'','precond':[],'steps':[],'isPrimitive':False}
        #TODO case if precondition are met already-> do anyway?
        self.goal=goal
        self.posAgents=posAgents
        self.agent=None
        self.posBoxes = posBoxes
        self.box = None
        self.refScheme = {\
            'FullfillBoxGoal': {'name':'FullfillBoxGoal','precond':[self.agentAvl,self.boxAvl],'steps':['SelectBox','SelectAgent',],'isPrimitive':False},\
            #'MoveAgentToBox':{'name':'MoveAgentToBox','precond':[self.agentAvl],'steps':[],'isPrimitive':True},\
            #'MoveAgentWithBox':  {'name':'MoveAgentWithBox','precond':[self.agentAdjToBox],'steps':[],'isPrimitive':True},\
            'SelectBox':{'name':'SelectBox','precond':[self.inSameRoom],'steps':[self.reducePosBoxes(),self.weightBoxes(),self.selectBox()],'isPrimitive':True},\
            'WeightBoxes':{'name':'WeightBoxes','precond':[],'steps':[],'isPrimitive':True},\
            'SelectAgent':{'name':'SelectAgent','precond':[],'steps':[self.reducePosAgents(),'WeightAgents',self.selectAgent()],'isPrimitive':False},\
            'WeightAgents':{'name':'WeightAgents','precond':[],'steps':[self.getAgentWorkload,self.getAgentDistance(),self.weightAgents],'isPrimitive':True},\
            'MoveAgent':{'name':'MoveAgent','precond':[self.agentAvl],'steps':[],'isPrimitive':True}
        }
        self.steps = [self.refScheme[headTask]]
        self.weight=0
        #start refinement loop
        self.tasks = []

    #
    def refine(self):
        self.steps = [self.refScheme[y] for x in self.steps for y in (self.refScheme[x['name']]['steps'] if x['name'] in self.refScheme and x['isPrimitive']==False  else [x]) if isinstance(y,str)]
        return self
    def allPrimitive(self):
        return all([i['isPrimitive'] for i in self.steps])
    #effects and preconditions
    def agentAvl(self):
        return True
    def boxAvl(self):
        return True
    def agentAdjToBox(self):
        return True
    def inSameRoom(self):
        return True
    def isSameColor(self):
        return True
    #mapping to real actions if all actions are Primitive
    def generateActions(self):
        if self.allPrimitive():
            pass
            #find actions 
    def selectAgent(self):
        self.agent=self.posAgents[0]
        print('selected Agent:'+str(self.agent))
    def weightAgents(self):
        #height weight means workload high and far away
        #TODO implement
        #use a heuristic to calc a weight
        pass
    def selectBox(self):
        self.box=self.posBoxes[0]
    def weightBoxes(self):
        #hight weight means far away
        #TODO implement
        #use a heuristic to get a weight
        pass
    def getAgentWorkload(self):
        #Workload amount of goals or amount of steps of already assigned tasks
        pass
    def getAgentDistance(self):
        for a in self.posAgents:
            print(a,file=sys.stderr,flush=True)
    def reducePosAgents(self):
        #check if agent can reach goal ->same room
        pass
    def reducePosBoxes(self):
        #check if box can reach goal -> same room
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
