#import argparse
#import re
import sys
import traceback
from state import StateSA,StateMA,StateBuilder
from problemDecomposer import problemDecomposer as pd
from coordinator import Coordinator
from action import north,south,west,east,move,push,pull
import os  


class SearchClient:
    def __init__(self, server_messages,):
        if not server_messages:
            return
        #Adapt to data structure and split msg in parts.
        self.domain = None
        self.levelname = None
        colors = {}
        init = []
        goal = []
        color_count = 0
        try:
            line = server_messages.readline().rstrip()
            case = 0
            while line:
                if line == "#domain":
                    case =1
                    next
                elif line == "#levelname":
                    case =2
                    next
                elif line == "#colors":
                    case =3
                    next
                elif line == "#initial":
                    case =4
                    next
                elif line == "#goal":
                    case =5
                    next
                elif line.rstrip() == "#end":
                    break
                else:
                    if case == 1:
                        self.domain = line
                    elif case == 2:
                        self.levelname = line
                    elif case == 3:
                        temp = line.split(':')
                        temp2 = temp[1].rstrip().split(',')
                        for i in temp2:
                            colors[i.strip()]= color_count
                        color_count+=1
                    elif case == 4:
                        init.append(line)
                    elif case == 5:
                        goal.append(line)
                #print(line,file=sys.stderr,flush=True)
                line = server_messages.readline().rstrip()

        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            print(traceback.format_exc(), file=sys.stderr, flush=True)
            sys.exit(1)

        cols = max([len(line) for line in init])
        maze = [[True for _ in range(cols)] for _ in range(len(init))]
        agent = []
        boxes = []
        goals = []
        type_count = 0
        seen_types = {}
        row = 0
        for line in init:
            for col, char in enumerate(line):
                if char == '+':
                    maze[row][col] = False
                    
                elif char in "0123456789":
                    agent_id = int(char)
                    agent_spec = ((row, col),colors[char])
                    agent.insert(agent_id, agent_spec)
                elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                    type = type_count
                    if char.lower() in seen_types.keys():
                        type = seen_types[char.lower()]
                    else:
                        seen_types[char.lower()] = type
                        type_count += 1
                    boxes.append((type, (row, col),colors[char]))
                elif char == ' ':
                    # Free cell.
                    pass
                else:
                    print('Error, read invalid level character: {}'.format(char), file=sys.stderr, flush=True)
                    sys.exit(1)
            row += 1
        row = 0
        for line in goal:
            for col, char in enumerate(line):
                if char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                    type = type_count
                    if char.lower() in seen_types.keys():
                        type = seen_types[char.lower()]
                    else:
                        seen_types[char.lower()] = type
                        type_count += 1
                    goals.append((type, (row, col)))
            row += 1

        self.initial_state = StateMA(maze,boxes,goals,agent)

        self.sendComment("Initialized SearchClient")

    def solve_the_problem(self):
        coordinator = Coordinator(self.initial_state)
        master_plan = coordinator.solve()
        for action_vector in master_plan:
            self.sendJointAction(action_vector)

    '''
    send joints action
    format of actions : {agent:action}
    agent - int
    action - string acording to assignment sheet e.g "Push(move-dir-agent, move-dir-box)"
    output to server <action0>; <action1>; ...; <action9>
    example:
        success = client.sendJointAction({0:"Move(E)",1:"Move(E)"})
    return array of bools for every action in the actions dict. bool describing the success of the action

    '''
    def sendJointAction(self,actions):

        jointAction = ";".join([str(action) if action else "NoOp" for action in actions])
        sys.stdout.write(jointAction+"\n")
        sys.stdout.flush()


        success = [i.rstrip() == "true" for i in sys.stdin.readline().rstrip().split(";")]
        return success

    def sendComment(self,comment):
        sys.stdout.write("#"+str(comment)+"\n")
        sys.stdout.flush()

def main():
    sys.stdout.write("GroupName\n")
    sys.stdout.flush()


    #If you supply a hard coded file name, it will run that hard coded level instead of
    #reading from the server. I can't find out how to pass command line arguments when
    #i use the debugger.... Sorry if this caused you to look around for a while in confusion :D
    hard_coded_file_name = None
    #hard_coded_file_name = "src/levels/chokepoint.lvl"
    #hard_coded_file_name = "../levels/chokepoint.lvl"

    #If a filename is passed as argument, we read directly from file instead of
    #using the server. Allows us to run debugger at the same time
    if len(sys.argv) >= 2:
        arg1 = sys.argv[1]
        if  os.path.isfile(arg1):
            server_messages = open(sys.argv[1])
            client = SearchClient(server_messages)
            server_messages.close()
        elif arg1=='-htn':
            server_messages = sys.stdin
            client = SearchClient(server_messages)
            client.solve_the_problem()
        else:
            raise ValueError("argument is not a solver")

        '''
        elif arg1=='-xxx':
            server_messages = sys.stdin
            client = SearchClient(server_messages)
            client.solve_the_problem()
        '''
        


    elif hard_coded_file_name:
        server_messages = open(hard_coded_file_name)
        client = SearchClient(server_messages)
        server_messages.close()
        #Follow this to get where the planning happens
        client.solve_the_problem()

    else:
        server_messages = sys.stdin
        client = SearchClient(server_messages)
        #Follow this to get where the planning happens
        client.solve_the_problem()

    
    #This will probably be moved at some point
    problem = pd(client.initial_state)
    tasks = problem.getTasks()
    print(tasks,file= sys.stderr, flush=True)

    #Follow this to get where the planning happens
    #client.solve_the_problem()

if __name__ == '__main__':
    main()
