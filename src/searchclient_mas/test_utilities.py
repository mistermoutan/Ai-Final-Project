from action import *
import copy
from state import StateMA


#agents are represented by "a:id:color"
#boxes  are represented by "b:type:color"
#goals  are represtend  by "g:type"

def agent(id,color):
    return "a:"+str(id)+":"+str(color)

def box(type,color):
    return "b:"+str(type)+":"+str(color)

def goal(type):
    return "g:"+str(type)

def make_state(matrix):
    matrix = copy.deepcopy(matrix)
    boxes  = []
    goals  = []
    agents = []
    

    for i,row in enumerate(matrix):
        for j,s in enumerate(row):
            #Wall or empty space in matrix (False or True)
            if not isinstance(s, str):
                continue
            
            e = s.split(":")
            #Agent
            if e[0] == "a": 
                agents.append((e[1],(i,j),e[2]))
            #Box
            elif e[0] == "b":
                boxes.append((e[1], (i,j), e[2]))
            #AGoal
            elif e[0] == "g":
                goals.append((e[1], (i,j)))
            else: 
                raise AssertionError("Cannot recognize " + e[0] + " as an agent, box or goal")
            
            #There is no wall at (i,j)
            matrix[i][j] = True
    
    #Sort the agents by identity. This happens automatically
    #id is first element in tuple
    agents.sort()
    agents = [(pos,color) for (id,pos,color) in agents]

    return StateMA(matrix, boxes, goals, agents)
