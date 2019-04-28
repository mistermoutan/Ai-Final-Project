from state import StateMA,StateBuilder
from typing import List, Tuple
import sys
from graph import Graph
from path_finder import actions_to_move_between,actions_to_push_box_between,actions_to_move_to_target_while_pulling_box,first_off_path_node
from action import move,push,pull,north,south,east,west
from master_plan import MasterPlan
from planner import Planner
from level_analyser import LevelAnalyser
from goal_analyzer import GoalAnalyzer
from action import NOOP

#from coordinator import Coordinator

'''
idea
    - get list of tasks/problems
    - decompose to primitive tasks
    - use planner plan pathes for tasks
    - weight tasks
    - sort tasks by weighting
    - Use Planner to decompose tasks to actions
    - send actions
'''
'''
refine until it is primitive
'''
class HTN():
    #TODO implement decisions for seperate rooms
    #TODO implement decisions for boxes and goals with the same name
    #TODO implement blocking corridor
    #TODO goal ordering in the actual plan...
    #TODO partly parallel partly sequetially
    #TODO 1. implement sequential order execution
    #TODO 2. get viable goals and execute them in parrall before the sequential part
    def __init__(self,state):
        self.state = state
        self.Tasks = []
        self.pd = problemDecomposer(state)
        self.graph_of_level = Graph(state.maze)
        self.number_of_agents = len(self.state.agent_positions)
        self.agent_workload = [0] * self.number_of_agents
        self.boxes_used =[]
        self.aproximated_agent_pos=self.state.agent_positions
        self.aproximate=True
        self.ga = GoalAnalyzer(self.state)
        self.goal_in_order = self.ga.compute_goal_order_plan()
        self.goals_par = []
        self.goals_seq = []
        #,file=sys.stderr,flush= True)
        #self.level_analyser = LevelAnalyser(self.state)
        #self.separated_rooms_exisit = self.level_analyser.separate_rooms_exist()
        #print(self.level_analyser.separate_rooms_exist(),file=sys.stderr,flush=True)
        #print(self.graph_of_level.)
    def createTasks(self):
        #for i in range(len(self.state.goal_types)):
        for idx,i in enumerate(self.goal_in_order):
            #TODO if goal type is box goal
            boxes = self.pd.searchPossibleBoxesForGoalIndex(i)
            print(boxes,file=sys.stderr,flush=True)
            agents= self.pd.searchPossibleAgentsForBoxIndex(boxes[0])
            print(agents,file=sys.stderr,flush=True)

            #TODO implement precondition data structure
            self.Tasks.append(Task('FullfillBoxGoal',i,self.state,self.graph_of_level,boxes,agents))
            self.Tasks[i].setOrder(idx)
            #add agent workload for task to global workload of agent
            #self.boxes_used.append(self.Tasks[i].box)
            #self.agent_workload= [self.agent_workload[k]+self.Tasks[i].workload[k] for k in range(len(self.Tasks[i].workload))]
    def refineTasks(self):
        '''
        refines given Tasks until they are primitive
        '''
        for i,t in enumerate(self.Tasks):
            t.setWorkload(self.agent_workload)
            t.setUsedBoxes(self.boxes_used)
            t.setAgentPos(self.aproximated_agent_pos)
            #if self.separated_rooms_exisit:
            #    t.setRoom(0)
            while not t.allPrimitive():
                t.refine()
            if t.headTask == 'FullfillBoxGoal':
                self.boxes_used.append(self.Tasks[i].box)
                self.agent_workload= [self.agent_workload[k]+self.Tasks[i].workload[k] for k in range(len(self.Tasks[i].workload))]
                #if (self.Tasks[i].agent != None):
                #print('agent:' +str(self.Tasks[i].agent),file=sys.stderr,flush=True)
                #print('aprx_agent_pos:' +str(self.aproximated_agent_pos[self.Tasks[i].agent]),file=sys.stderr,flush=True)
                #print('state_goal_pos:' +str(self.state.goal_positions[self.Tasks[i].goal]),file=sys.stderr,flush=True)
                self.updateAgentsPos(self.Tasks[i].agent,self.state.agent_positions[self.Tasks[i].agent])
                #self.aproximated_agent_pos[self.Tasks[i].agent]=self.state.goal_positions[self.Tasks[i].goal]
    def updateAgentsPos(self,agentID,newPos):
        self.aproximated_agent_pos[agentID]=newPos
    def sortTasks(self):
        '''
        Sorts Tasks by its weight
        TODO implement weight funciton
        '''
        self.Tasks = sorted(self.Tasks, key=lambda k: k.order,reverse=True)

    def getTasksByAgent(self):
        '''
        returns a dict of actions that is assigned to solve tasks
        !!! number of keys must not be equal to the number of agents
        '''
        agentTask={i:[] for i in range(len(self.state.agent_positions))}
        #agentTask = {}
        for t in self.Tasks:
            if t.agent not in agentTask:
                agentTask[t.agent]=[(t.goal,t.box)]
            else:
                agentTask[t.agent].append((t.goal,t.box))
        #Workaround because the agent order matters
        return {key: agentTask[key]for key in sorted(agentTask.keys())}
    def getSingleTaskByAgent(self,task):
        return (task.agent,[(task.goal,task.box)])
    def distance_to(self, x, y):
        #changed this to resolve error
        path_length = len(self.graph_of_level.shortest_path_between(x,y))
        if path_length > 0:
            return path_length-1
        else:
            return path_length
    def min_distance_to_position_in_list(self, box, goals):
        distances = [self.distance_to(box, goal) for goal in goals]
        return min(distances)
    def distances_to_position_in_list(self, pos1, poslist):
        return [self.distance_to(pos1, pos2) for pos2 in poslist]
    def ind_n_dis_goals_to_closest_box(self, state, boxes, goals):
        '''
        for every goal return the index of closest box that can satisfy that goal
        and is not assigned to another goal already, and the distance to it
        (we have to calculate the distance anyway so we might aswell keep it)
        '''
        closest_boxes = []
        distances = []

        for i, goal in enumerate(goals):
            d = []
            for j, box in enumerate(boxes):
                if j not in closest_boxes:
                    if state.goal_types[i] == state.box_types[j]:
                        d.append(self.distance_to(box, goal))
                    else:
                        d.append(2500)
                else:
                    d.append(2500)

            min_index = d.index(min(d))
            closest_boxes.append(min_index)
            distances.append(d[min_index])

        return closest_boxes, distances

        def heuristic_adv(self, state, alpha = 1):
            '''
            Idea is to combine different heurisitcs here and wheight them differently, sqaure them, etc.
            Work in progress
            '''
            agent = (state.agent_row, state.agent_col)
            boxes = state.box_positions
            goals = state.goal_positions
            alpha = 5.5 #penalizing factor for distance goals_to_box
            square_goals2box = False #True: goals will be solved almost in "parrallel" / False: #boxes will be pushed to their goals "sequentially"
            square_agt2boxes = False # not really helpful
            goal_reward = 100 # additional reward to keep box at goal

            #closest box for every goal and the distance to it
            closest_boxes, dist_goals_to_box = self.ind_n_dis_goals_to_closest_box(state, boxes, goals)

            #distances form agent to all boxes that are not in goal state
            dist_agent_to_boxes = self.distances_to_position_in_list(agent, [boxes[cb] for i,cb in enumerate(closest_boxes) if dist_goals_to_box[i] != 0])
            dist_agent_to_boxes = [d-1 for d in dist_agent_to_boxes] #currently error of 1 #TODO resolve this?

            #reward for solved goals
            goal_reward = goal_reward*len(set(dist_goals_to_box)-set([0]))

            # not enough boxes for goals
            if 2500 in dist_goals_to_box:
                raise ValueError("Not enough boxes to satisfy all goals")

            #avoid error in goal state - every goal is satisfied
            if dist_agent_to_boxes == []:
                dist_agent_to_boxes = [0]

            if square_goals2box:
                dist_goals_to_box = [d*d for d in dist_goals_to_box]

            if square_agt2boxes:
                dist_agent_to_boxes = [d*d for d in dist_agent_to_boxes]

            #print("dist_goals_to_box: {}".format(dist_goals_to_box))
            #print("dist_goals_to_box: {}, dist_agent_to_boxes: {} ".format(dist_goals_to_box, dist_agent_to_boxes), file= sys.stderr,flush=True)
            h = alpha * sum(dist_goals_to_box) + min(dist_agent_to_boxes) + goal_reward + state.g

            return h

    def make_single_agent_plan(self, initial_state):
        return Planner(initial_state, heuristic=self.heuristic_adv, g_value=lambda x: 1,cutoff_solution_length=30).make_plan()
    def make_single_agent_plan_empty(self,initial_state):
        return Planner(initial_state)
    def solve_par(self):
        self.createTasks()
        self.refineTasks()
        self.sortTasks()
        single_agent_tasks = self.getTasksByAgent()
        #print('sat'+str(single_agent_tasks),file=sys.stderr,flush=True)
        #run all low level functions to create primitive tasks

        #create all single agents states to
        single_agent_states = [self.state.get_HTN_StateSA(agent,tasks,True) for agent,tasks in single_agent_tasks.items()]


        #single_agent_states = [self.state.get_HTN_StateSA(i,single_agent_tasks[i]) for i in  range(number_of_agents)]
        #create single agent plans
        single_agent_plans = [self.make_single_agent_plan(s) for s in single_agent_states]
        #create masterplan
        master_plan = MasterPlan(self.number_of_agents, self.state)
        #merge singleAgentPlans into MasterPlan
        for agent,plan in enumerate(single_agent_plans):
            master_plan.merge_plan_into_master(agent,plan)
        #workaround add empty plans for unused agents
        return master_plan.plan
    def plan_for_agent_where_other_agents_are_waiting(self, agent_id, agent_plan):
        #Make an action vector where no agent does anything/Users/ek/Downloads/sequentialized_plans_hack.py
        empty_action_vector = [NOOP] * self.number_of_agents

        #Make an empty plan of the same length as parameter agent_plan, where no agent does anything
        empty_plan = [empty_action_vector.copy() for i in agent_plan]

         #Put the plan for the agent agent_id into the empty action plan
        for i,action_vector in enumerate(empty_plan):
            action_vector[agent_id] = agent_plan[i]

        return empty_plan
    def solve_seq(self):
        self.createTasks()
        self.refineTasks()
        self.sortTasks()
        single_agent_tasks = [self.getSingleTaskByAgent(t) for t in self.Tasks]
        #print('sat'+str(single_agent_tasks),file=sys.stderr,flush=True)
        #run all low level functions to create primitive tasks
        #create sequential multi agent plans
        multi_agent_plans_seq = [self.plan_for_agent_where_other_agents_are_waiting(agent,self.make_single_agent_plan(self.state.get_HTN_StateSA(agent,tasks,True))) for agent,tasks in single_agent_tasks]
        final_plan = []
        for i in multi_agent_plans_seq:
            final_plan+=i
        #print(final_plan)
        return final_plan
class Task():
    #TODO Choose between agents based on current Workload and Distance
    #TODO Availablity of agents - init selection function in Task but run it in HTN if all Tasks are created
    #TODO Choose between boxes
    def __init__(self,headTask,goal,state,graph,posBoxes=[],posAgents=[]):
        #refinement schema for all actions
        #TODO make sure that headTask is in refScheme -> exception handling
        #schema = {'name':'','precond':[],'steps':[],'isPrimitive':False}
        #TODO case if precondition are met already-> do anyway?
        self.state = state
        self.graph = graph
        self.goal=goal
        self.posAgents=posAgents
        self.agent=None
        self.posBoxes = posBoxes
        self.box = None
        self.workload = []
        self.refScheme = {\
            'FullfillBoxGoal': {'name':'FullfillBoxGoal','precond':[self.agentAvl,self.boxAvl],'steps':['SelectBox','SelectAgent',self.getAgentBoxDistance],'isPrimitive':False},\
            'SelectBox':{'name':'SelectBox','precond':[],'steps':[self.reducePosBoxes,self.weightBoxes,self.selectBox],'isPrimitive':True},\
            'SelectAgent':{'name':'SelectAgent','precond':[],'steps':[self.reducePosAgents,'WeightAgents'],'isPrimitive':False},\
            'WeightAgents':{'name':'WeightAgents','precond':[],'steps':[self.getAgentWorkload,self.selectAgent],'isPrimitive':True},\
            #'MoveAgentToBox':{'name':'MoveAgentToBox','precond':[self.agentAvl],'steps':[],'isPrimitive':True},\
            #'MoveAgentWithBox':  {'name':'MoveAgentWithBox','precond':[self.agentAdjToBox],'steps':[],'isPrimitive':True},\
            #'WeightBoxes':{'name':'WeightBoxes','precond':[],'steps':[],'isPrimitive':True},\
            'FullfillAgentGoal':{'name':'FullfillAgentGoal','precond':[],'steps':['selectAgent'],'isPrimitive':True},\
            #'MoveAgent':{'name':'MoveAgent','precond':[self.agentAvl],'steps':[],'isPrimitive':True}
        }
        self.steps = [self.refScheme[headTask]]
        self.order=0
        self.agents_weight={}
        self.headTask = headTask
        #start refinement loop

    #
    def setWorkload(self,workload):
        self.workload = workload
    def setUsedBoxes(self,used_boxes):
        self.used_boxes=used_boxes
    def setAgentPos(self,agent_pos):
        self.agent_pos=agent_pos
    def setRoom(self,room_id):
        self.room=room_id
    def setOrder(self,order):
        self.order = order
    def refine(self):
        #self.steps = [self.refScheme[y] for x in self.steps for y in (self.refScheme[x['name']]['steps'] if x['name'] in self.refScheme and x['isPrimitive']==False  else [x]) if isinstance(y,str)]
        temp_steps=[]
        for x in self.steps:
            if (x['name'] in self.refScheme) and (x['isPrimitive']==False):
            #check preconditions
                if all ([precond() for precond in x['precond']]):
                    for y in (self.refScheme[x['name']]['steps']):
                        if isinstance(y,str):
                            temp_steps.append(self.refScheme[y])
                        else:
                            if callable(y):
                                y()
            elif(x['isPrimitive']==True):
                for y in (self.refScheme[x['name']]['steps']):
                    if callable(y):
                        y()

        self.steps = temp_steps
        #print(self.steps,file=sys.stderr,flush=True)
        return self
    def allPrimitive(self):
        #return all([i == True or i['isPrimitive'] for i in self.steps])
        return len(self.steps)==0

    #effects and preconditions
    def agentAvl(self):
        return True if len(self.posAgents)>0 else False
    def boxAvl(self):
        return True if len(self.posBoxes)>0 else False
    def agentAdjToBox(self):
        return True
    def inSameRoom(self):
        return True
    #mapping to real actions if all actions are Primitive
    def selectAgent(self):
        self.agent = self.best_agentBox_combi[0]
    def reducePosAgents(self):
        #reduce to agents that are in the same room
        pass
    def selectBox(self):
        self.box = self.best_agentBox_combi[1]
    def weightBoxes(self):
        #hight weight means far away
        #TODO implement
        #use a heuristic to get a weight
        pass
    def reducePosBoxes(self):
        self.posBoxes  = [x for x in self.posBoxes if x not in self.used_boxes]
        #reduce to boxes that are in the same room
    def getAgentWorkload(self):
        return self.workload if hasattr(self,'workload') else []
    def getAgentDistance(self):
        if not hasattr(self,'agent_pos'):
            self.agent_pos = self.state.agent_positions
        goal_pos = self.state.goal_positions[self.goal]
        for a in self.posAgents:
            if a in self.agents_weight:
                self.agents_weight[a]+=self.distance_to(self.agent_pos[a],goal_pos)
            else:
                self.agents_weight[a]=self.distance_to(self.agent_pos[a],goal_pos)
    def getAgentBoxDistance(self):
        if not hasattr(self,'agent_pos'):
            self.agent_pos = self.state.agent_positions
        self.agentBox_combi = {}
        goal_pos = self.state.goal_positions[self.goal]
        box_pos = self.state.box_positions
        for a in self.posAgents:
            for b in self.posBoxes:
                if (a,b) in self.agentBox_combi:
                    self.agentBox_combi[(a,b)]+=self.distance_to(self.agent_pos[a],box_pos[b])
                    self.agentBox_combi[(a,b)]+=self.distance_to(box_pos[b],goal_pos)
                else:
                    self.agentBox_combi[(a,b)]=self.distance_to(self.agent_pos[a],box_pos[b])
                    self.agentBox_combi[(a,b)]+=self.distance_to(box_pos[b],goal_pos)

        #best combi
        self.best_agentBox_combi=min(self.agentBox_combi, key=self.agentBox_combi.get)
    #Help Functions
    def distance_to(self, x, y):
        return len(self.graph.shortest_path_between(x,y))

'''
This class decomposes a level into high level problems where tasks can be created based
on possible agents that can bring possible boxes to specific goals
'''
class problemDecomposer():
    def __init__(self,state):
        self.Tasks  = []
        self.Plan = []
        self.state = state
        self.level_analyser = LevelAnalyser(self.state)
        self.separated_rooms_exisit = self.level_analyser.separate_rooms_exist()
        if self.separated_rooms_exisit:
            self.level_analyser.get_agent_distribution_per_room()
            self.agents_per_room = self.level_analyser.agents_per_room
            self.level_analyser.get_box_distribution_per_room()
            self.boxes_per_room = self.level_analyser.boxes_per_room
            self.level_analyser.get_goals_distribution_per_room()
            self.goals_per_room = self.level_analyser.goals_per_room
            #print('agents_per_room'+str(self.agents_per_room),file=sys.stderr,flush=True)
            #print('boxes_per_room'+str(self.boxes_per_room),file=sys.stderr,flush=True)
            #print('goals_per_room'+str(self.goals_per_room),file=sys.stderr,flush=True)

        #print(self.state.box_types,file= sys.stderr, flush=True)

    def get_room(self,room_cor,_dict):
        for room, value in _dict.items():
            if  value !=None:
                if room_cor in value:
                    return room
        return None
    def getGoalOrientedProblems(self):
        '''
        '''
        g = []
        for i in range(len(self.state.goal_types)):
            g.append((i,self.searchPossibleBoxesForGoalIndex(i)))
        self.Plan.insert(g)
        return self.Plan
    def searchPossibleAgentsForBox(self,box_idx):
        '''
        returns a dict of agent indexes that are able to move the box at pos box_idx in the box list
        and adds an initial value for the weight of an agent
        '''
        #TODO in same room?
        return {idx:0 for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]}

    def searchPossibleBoxesForGoalIndex(self,goal_idx):
        '''
        returns a list of box-indexes that are able to satisfy the goal at pos goad_idx in the goal list
        '''
        if self.separated_rooms_exisit:
            goal_room = self.get_room(self.state.goal_positions[goal_idx],self.goals_per_room)
            return [idx for idx in range(len(self.state.box_types)) if self.state.goal_types[goal_idx] ==self.state.box_types[idx] and self.state.box_positions[idx] in self.boxes_per_room[goal_room]]
        else:
            return [idx for idx in range(len(self.state.box_types)) if self.state.goal_types[goal_idx] ==self.state.box_types[idx]]

    def searchPossibleGoalsForBoxIndex(self,box_idx):
        '''
        returns a list of goal-indexes that are able to store the box at index box_idx
        '''
        #TODO in same room?
        return [idx for idx in range(len(self.state.goal_types)) if self.state.goal_types[idx] ==self.state.box_types[box_idx]]

    def searchPossibleAgentsForBoxIndex(self,box_idx):
        '''
        returns a list of agent indexes that are able to move the boxes at pos i in the box list
        '''
        if self.separated_rooms_exisit:
            #get room of goal
            box_room = self.get_room(self.state.box_positions[box_idx],self.boxes_per_room)
            return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx] and idx in self.agents_per_room[box_room]]
        else:
            return [idx for idx in range(len(self.state.agent_colors)) if self.state.box_colors[box_idx]==self.state.agent_colors[idx]]

    def assign_tasks_greedy(self):
        '''
        - greedely assings tasks to agents with lowest workload that can fullfill the task
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
        '''
        idea:
        - for a goal in all goals
            get all boxes that can satisfy that goal
            get distance from goal to every such box
            get distance of all possible agents for each such box and add current workload of the agent
            sum both, take lowest
            remove assigend box from list, add workload to agent
        - repeat for all goals
        '''

        self.agt_tasks = [[] for i in range(len(self.state.agent_colors))]

        #first step: assign boxes to goals--------------------------------------
        print(self.state.box_types)
        assigned_boxes =[]

        wrkld = [0]*len(self.state.agent_colors)

        for i in range(len(self.state.goal_types)):

            print("-----------------------")
            print(i)
            print("current goal pos: {}".format(self.state.goal_positions[i]))

            #get all possible boxes that could satisfy current goal
            psbl_boxes = list(set(self.searchPossibleBoxesForGoalIndex(i))-set(assigned_boxes))
            psbl_boxes_pos = [self.state.box_positions[j] for j in psbl_boxes]
            print("possible box pos: {}".format(psbl_boxes_pos))

            added_dists = coordi.distances_to_position_in_list(self.state.goal_positions[i],psbl_boxes_pos)
            print("added_dists: {}".format(added_dists))

            assigned_agents = []
            #get all possible agents for each box in possible boxes
            for k,pb in enumerate(psbl_boxes):
                #get distance to agents for box
                psbl_agents = self.searchPossibleAgentsForBoxIndex(pb)
                psbl_agents_pos = [self.state.agent_positions[n] for n in psbl_agents]
                dists_box_2_agents = coordi.distances_to_position_in_list(self.state.box_positions[pb],psbl_agents_pos)

                #get current workload for all possible agents
                psbl_wrkld = [wrkld[w] for w in psbl_agents]

                #add workload to distance
                dists_wrkld = [x+y for x,y in zip(dists_box_2_agents, psbl_wrkld)]

                #take min of distance+workload (fastest agent to fullfill goal)
                min_idx = dists_wrkld.index(min(dists_wrkld))

                #this agent takes the task
                assigned_agents.append(psbl_agents[min_idx])

                #update the distances
                added_dists[k] += dists_wrkld[min_idx]

            assigned_boxes.append(psbl_boxes[added_dists.index(min(added_dists))])

            #update workload of the agent that finally takes the task
            assigned_agt = assigned_agents[added_dists.index(min(added_dists))]
            wrkld[assigned_agt] +=  added_dists[k]+dists_box_2_agents[min_idx]

            self.agt_tasks[assigned_agt].append(assigned_boxes[i])

        print(self.agt_tasks)
        print("done, assinged boxes for goals: {}".format(assigned_boxes))

        return self.agt_tasks

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
