#import argparse
#import re
import sys
import traceback
from state import StateSA



class SearchClient:
    def __init__(self, server_messages):
        #Adapt to data structure and split msg in parts.
        self.domain = None
        self.levelname = None
        self.colors = {}
        self.init = []
        self.goal = []
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
                        self.colors[temp[0]]= temp[1]
                    elif case == 4:
                        self.init.append(line)
                    elif case == 5:
                        self.goal.append(line)
                #store msg in datastructure
                #print(line, file=sys.stderr, flush=True)
                #if line.rstrip()=="#end":
                #    break
                line = server_messages.readline().rstrip()

        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            print(traceback.format_exc(), file=sys.stderr, flush=True)
            sys.exit(1)
        
        cols = max([len(line) for line in self.init])
        maze = [[True for _ in range(cols)] for _ in range(len(self.init))]
        agent = (0,0)
        boxes = []
        goals = []
        type_count = 0
        seen_types = {}
        row = 0
        for line in self.init:
            for col, char in enumerate(line):
                if char == '+':
                    maze[row][col] = False
                elif char in "0123456789":
                    agent = (row, col)
                elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                    type = type_count
                    if char.lower() in seen_types.keys():
                        type = seen_types[char.lower()]
                    else:
                        seen_types[char.lower()] = type
                        type_count += 1
                    boxes.append((type, (row, col)))
                elif char in "abcdefghijklmnopqrstuvwxyz":
                    type = type_count
                    if char.lower() in seen_types.keys():
                        type = seen_types[char.lower()]
                    else:
                        seen_types[char.lower()] = type
                        type_count += 1
                    goals.append((type, (row, col)))
                elif char == ' ':
                    # Free cell.
                    pass
                else:
                    print('Error, read invalid level character: {}'.format(char), file=sys.stderr, flush=True)
                    sys.exit(1)
            row += 1
        self.initial_state = StateSA(maze, boxes, goals, agent)
        self.sendComment("Initialized SearchClient")
    def search(self, strategy: 'Strategy'):
        pass
    def sendAction(self,action):
        sys.stdout.write(str(action)+"\n")
        sys.stdout.flush()
    def sendComment(self,comment):
        sys.stdout.write("#"+str(comment)+"\n")
        sys.stdout.flush()
    
def main():
    #implement parse aguments if different planing algorithms are planned
    sys.stdout.write("ExampleClient\n")
    sys.stdout.flush()
    server_messages = sys.stdin
    client = SearchClient(server_messages)
if __name__ == '__main__':
    # Run client.
    main()

