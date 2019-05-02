import random
from collections import defaultdict
from action import ALL_ACTIONS, ActionType, Action, Dir
from typing import List, Tuple
import sys


class StateSA:
    _RNG = random.Random(1)

    def __init__(self, maze: List[List[int]] = None, boxes: List[Tuple[int, Tuple[int, int]]] = None,
                 goals: List[Tuple[int, Tuple[int, int]]] = None, agent: Tuple[int, int] = None):
        '''
        :param maze: maze should be a grid containing true or false with false being walls and true being open spaces
        :param boxes: boxes should be a list containing the following tuple: (type (number), position (x,y))
        :param goals: goals should be a list containing the following tuple: (type (number), position (x,y))
        :param agent: tuple of agent's position (x,y)
        '''
        if maze is None:
            return

        self.rows = len(maze)
        self.cols = len(maze[0])
        self.maze = maze

        self.box_types = [b[0] for b in boxes]
        self.box_positions = [b[1] for b in boxes]
        self.box_by_cords = {pos: i for i, pos in enumerate(self.box_positions)}

        self.goal_types = [b[0] for b in goals]
        self.goal_positions = [b[1] for b in goals]
        self.goal_by_cords = {pos: i for i, pos in enumerate(self.goal_positions)}

        self.agent_row = agent[0]
        self.agent_col = agent[1]

        self.parent = None
        self.g = 0
        self.action = None

        self._hash = None

        self.action_performed = None

    def copy(self):
        # shallow copy for more efficient get children, especially if agent is only being moved
        cpy = StateSA()
        cpy.rows = self.rows

        cpy.cols = self.cols
        cpy.maze = self.maze

        cpy.box_types = self.box_types
        cpy.box_positions = self.box_positions
        cpy.box_by_cords = self.box_by_cords

        cpy.goal_types = self.goal_types
        cpy.goal_positions = self.goal_positions
        cpy.goal_by_cords = self.goal_by_cords
        cpy.agent_row = self.agent_row
        cpy.agent_col = self.agent_col

        cpy.parent = self.parent
        cpy.g = self.g
        cpy.action = self.action

        cpy._hash = self._hash

        return cpy

    def safe_box_move(self, old_pos, new_pos):
        """
        This function moves box from old pos to new_pos while being safe against
        shallow copies so that old states dont get ruined
        """

        # copy these variables so they are unaffected in previous states
        self.box_positions = [b for b in self.box_positions]
        self.box_by_cords = dict(self.box_by_cords)

        box_id = self.box_by_cords[old_pos]
        self.box_positions[box_id] = new_pos
        del self.box_by_cords[old_pos]
        self.box_by_cords[new_pos] = box_id

    def get_children(self) -> '[StateSA, ...]':
        '''
        Returns a list of child states attained from applying every applicable action in the current state.
        The order of the actions is random.
        '''
        children = []
        # print("\n\ncurr:", file=sys.stderr, flush=True)
        # print(str(self), file=sys.stderr, flush=True)
        # print("\nchildern:", file=sys.stderr, flush=True)
        for action in ALL_ACTIONS:
            # Determine if action is applicable.
            new_agent_row = self.agent_row + action.agent_dir.d_row
            new_agent_col = self.agent_col + action.agent_dir.d_col

            if action.action_type is ActionType.Move:
                if self.is_free(new_agent_row, new_agent_col):
                    child = self.copy()
                    child.agent_row = new_agent_row
                    child.agent_col = new_agent_col
                    child.parent = self
                    child.action = action
                    child.g += 1
                    child._hash -= hash((self.agent_row, self.agent_col))
                    child._hash += hash((new_agent_row, new_agent_col))
                    child.action_performed = action
                    # child._hash = None
                    children.append(child)
            elif action.action_type is ActionType.Push:
                if self.box_at(new_agent_row, new_agent_col):
                    new_box_row = new_agent_row + action.box_dir.d_row
                    new_box_col = new_agent_col + action.box_dir.d_col
                    if self.is_free(new_box_row, new_box_col):
                        child = self.copy()
                        child.agent_row = new_agent_row
                        child.agent_col = new_agent_col
                        child.safe_box_move((new_agent_row, new_agent_col), (new_box_row, new_box_col))
                        child.parent = self
                        child.action = action
                        child.g += 1
                        child._hash = None
                        child.action_performed = action
                        # print(str(child), file=sys.stderr, flush=True)
                        children.append(child)
            elif action.action_type is ActionType.Pull:
                if self.is_free(new_agent_row, new_agent_col):
                    box_row = self.agent_row + action.box_dir.d_row
                    box_col = self.agent_col + action.box_dir.d_col
                    if self.box_at(box_row, box_col):
                        child = self.copy()
                        child.agent_row = new_agent_row
                        child.agent_col = new_agent_col
                        child.safe_box_move((box_row, box_col), (self.agent_row, self.agent_col))
                        child.parent = self
                        child.action = action
                        child.g += 1
                        child._hash = None
                        child.action_performed = action
                        # print(str(child), file=sys.stderr, flush=True)
                        children.append(child)

        # for i in children:
        #    print(str(i), file=sys.stderr, flush=True)
        StateSA._RNG.shuffle(children)
        return children

    def is_initial_state(self) -> 'bool':
        return self.parent is None

    def is_goal_state(self) -> 'bool':
        # print("\n",str(self), file=sys.stderr, flush=True)
        for i in range(len(self.goal_types)):
            p = self.goal_positions[i]
            if p in self.box_by_cords:
                box_id = self.box_by_cords[p]
                if self.goal_types[i] != self.box_types[box_id]:
                    return False
            else:
                # print("failed: empty",(x,y), file=sys.stderr, flush=True)
                return False
        return True

    def is_free(self, row: 'int', col: 'int') -> 'bool':
        return self.maze[row][col] and (row, col) not in self.box_by_cords

    def box_at(self, row: 'int', col: 'int') -> 'bool':
        return (row, col) in self.box_by_cords

    def extract_plan(self) -> '[StateSA, ...]':
        plan = []
        state = self
        while not state.is_initial_state():
            plan.append(state.action_performed)
            state = state.parent
        plan.reverse()
        return plan

    def box_hash(self):
        return sum(hash((self.box_types[i], self.box_positions[i])) for i in range(len(self.box_types)))

    def change_goals(self, goals):
        # accepts goals of the same format as state constructor,
        # can be changed to just indices of goals or something else.
        cpy = self.copy()
        cpy.goal_types = [b[0] for b in goals]
        cpy.goal_positions = [b[1] for b in goals]
        cpy.goal_by_cords = {pos: i for i, pos in enumerate(self.goal_positions)}
        return cpy

    def __hash__(self):
        if self._hash is None:
            prime = 31
            _hash = 1
            _hash = _hash * prime + self.box_hash()
            _hash = _hash * prime + hash((self.agent_row, self.agent_col))
            self._hash = _hash
        return self._hash

    def __lt__(self, other):
        return False

    def __eq__(self, other):
        # print("equality check:\n",self,"\n",other, file=sys.stderr, flush=True)
        if self is other: return True
        if not isinstance(other, StateSA): return False
        if self.agent_row != other.agent_row: return False
        if self.agent_col != other.agent_col: return False
        for pos in self.box_positions:
            if pos not in other.box_by_cords:
                return False
            else:
                curr_id = self.box_by_cords[pos]
                other_id = other.box_by_cords[pos]
                if self.box_types[curr_id] != other.box_types[other_id]:
                    return False
        return True

    def __repr__(self):
        lines = []
        for row in range(self.rows):
            line = []
            for col in range(self.cols):
                pos = (row, col)
                if pos in self.box_by_cords:
                    line.append(str(self.box_types[self.box_by_cords[pos]]).upper())
                elif self.agent_row == row and self.agent_col == col:
                    line.append('0')
                elif pos in self.goal_by_cords:
                    line.append(str(self.goal_types[self.goal_by_cords[pos]]).lower())
                elif not self.maze[row][col]:
                    line.append('+')
                else:
                    line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)


class StateBuilder:
    def __init__(self):
        self.goals = []
        self.agents = []
        self.boxes = []
        self.maze = []

    def set_maze(self, maze):
        self.maze = maze

    def add_wall(self, position):
        r,c = position
        self.maze[r][c] = False
        return self

    def add_agent(self, agent_id, position, color):
        self.agents.append((agent_id, position, color))
        return self

    def add_box(self, box_type, position, color):
        self.boxes.append((box_type,position,color))
        return self

    def add_goal(self, goal_type, position, agent_goal=False):
        self.goals.append((goal_type, position, agent_goal))
        return self

    def build_StateMA(self):
        assert len(self.maze) > 0, "no maze provided, cant build state"

        agents = [None]*len(self.agents)
        for id, pos, c in self.agents:
            agents[int(id)] = (pos, c)

        return StateMA(self.maze, self.boxes, self.goals, agents)

class StateMA:
    _RNG = random.Random(1)

    def __init__(self, maze: List[List[int]] = None, boxes: List[Tuple[int, Tuple[int, int], int]] = None,
                 goals: List[Tuple[int, Tuple[int, int]]] = None, agents: List[Tuple[Tuple[int, int], int]] = None):
        '''
        :param maze: maze should be a grid containing true or false with false being walls and true being open spaces
        :param boxes: boxes should be a list containing the following tuple: (type (number), position (x,y), color (number))
        :param goals: goals should be a list containing the following tuple: (type (number), position (x,y))
        :param agents: boxes should be a list containing the following tuple: (position (x,y), color (number))
        '''
        if maze is None:
            return

        self.rows = len(maze)
        self.cols = len(maze[0])
        self.maze = maze

        self.agent_positions = [a[0] for a in agents]
        self.agent_colors = [a[1] for a in agents]
        self.agent_by_cords = {pos: i for i, pos in enumerate(self.agent_positions)}

        movable_colors = set()
        for color in self.agent_colors:
            movable_colors.add(color)

        curr = 0
        self.box_types = []
        self.box_positions = []
        self.box_colors = []
        self.box_by_cords = {}
        maze_safe = False
        # any boxes that mismatch the color of all agents will be assumed walls
        # TODO: should this be done elsewhere maybe the level analyzer?
        for i in range(len(boxes)):
            type, pos, color = boxes[i]
            if color in movable_colors:
                self.box_by_cords[pos] = curr
                self.box_positions.append(pos)
                self.box_colors.append(color)
                self.box_types.append(type)
                curr += 1
            else:
                x, y = pos
                # since we are editing them aze we need to make sure it doesnt get broken in the caller's maze
                # TODO: make slightly more efficient by nto copying whole maze but only necessary parts?
                if not maze_safe:
                    self.maze = [[self.maze[j][i] for i in range(self.cols)] for j in range(self.rows)]
                    maze_safe = True
                self.maze[x][y] = False

        self.goal_types = [g[0] for g in goals]
        self.goal_positions = [g[1] for g in goals]
        self.goal_agent = [g[2] for g in goals]
        self.goal_by_cords = {pos: i for i, pos in enumerate(self.goal_positions)}

        self.parent = None
        self.g = 0
        self.action = None

        self._hash = None

    def in_bounds(self, pos):
        row, col = pos
        if row < self.rows and col < self.cols and row >= 0 and col >= 0:
            return self.maze[row][col]

    def is_free(self, row: 'int', col: 'int' = None) -> 'bool':
        if isinstance(row, tuple):
            row, col = row
        return self.maze[row][col] and \
                (row, col) not in self.box_by_cords and \
                (row, col) not in self.agent_by_cords

    def box_at(self, row: 'int', col: 'int') -> 'bool':
        return (row, col) in self.box_by_cords

    def set_agent_position(self, old_pos, new_pos):
        id = self.agent_by_cords[old_pos]
        self.agent_positions[id] = new_pos
        del self.agent_by_cords[old_pos]
        self.agent_by_cords[new_pos] = id

    def set_box_position(self, old_pos, new_pos):
        id = self.box_by_cords[old_pos]
        self.box_positions[id] = new_pos
        del self.box_by_cords[old_pos]
        self.box_by_cords[new_pos] = id


    def copy(self):
        # TODO make a shallow copy solution
        maze = self.maze
        boxes = [(i,j,k) for i,j,k in zip(self.box_types ,self.box_positions, self.box_colors)]
        goals = [(i,j,k) for i,j,k in zip(self.goal_types ,self.goal_positions, self.goal_agent)]
        agents = [(i,j) for i,j in zip(self.agent_positions ,self.agent_colors)]

        return StateMA(maze, boxes, goals, agents)

    def move(self, agent, dir):
        x0, y0 = self.agent_positions[agent]

        x1 = x0 + dir[0]
        y1 = y0 + dir[1]
        # returns a tuple of the format:
        # freed space, occupied space, agent from, agent to, box_id, box_from, box_to
        return ((x0, y0),(x1, y1),(x0, y0),(x1, y1), None, None, None)

    def push(self, agent, agent_dir, box_dir):
        agent_x, agent_y = self.agent_positions[agent]
        new_agent_x = agent_x + agent_dir[0]
        new_agent_y = agent_y + agent_dir[1]
        if not self.box_at(new_agent_x, new_agent_y):
            return False

        box_id = self.box_by_cords[(new_agent_x, new_agent_y)]

        if self.box_colors[box_id] != self.agent_colors[agent]:
            return False

        new_box_x = new_agent_x + box_dir[0]
        new_box_y = new_agent_y + box_dir[1]


        # returns a tuple of the format:
        # freed space, occupied space, agent from, agent to, box_id, box_from, box_to

        return ((agent_x, agent_y),
                (new_box_x, new_box_y),
                (agent_x, agent_y),
                (new_agent_x, new_agent_y),
                box_id,
                (new_agent_x, new_agent_y),
                (new_box_x, new_box_y)
                )


    def pull(self, agent, agent_dir, box_dir):
        agent_x, agent_y = self.agent_positions[agent]
        new_agent_x = agent_x + agent_dir[0]
        new_agent_y = agent_y + agent_dir[1]

        box_x = agent_x + box_dir[0]
        box_y = agent_y + box_dir[1]

        if not self.box_at(box_x, box_y):
            return False

        box_id = self.box_by_cords[(box_x, box_y)]

        if self.box_colors[box_id] != self.agent_colors[agent]:
            return False


        # returns a tuple of the format:
        # freed space, occupied space, agent from, agent to, box_id, box_from, box_to

        return ((box_x, box_y),
                (new_agent_x, new_agent_y),
                (agent_x, agent_y),
                (new_agent_x, new_agent_y),
                 box_id,
                (box_x,box_y),
                (agent_x, agent_y)
                )

    # this function us used by get child but is not safe to use in all cases because
    # agent_to and box_to will get over written regardless of what is there before
    def perform_action(self, agent_id, agent_from, agent_to, box_id, box_from, box_to):
        self.agent_positions[agent_id] = agent_to
        self.agent_by_cords[agent_to] = agent_id

        if self.agent_by_cords[agent_from] == agent_id:
            del self.agent_by_cords[agent_from]


        if box_id is None:
            return
        self.box_positions[box_id] = box_to
        self.box_by_cords[box_to] = box_id

        if self.box_by_cords[box_from] == box_id:
            del self.box_by_cords[box_from]




    def get_child(self, actions: List[Action]):
        # tracks which spaces are occupied
        occupation_dict = {}
        action_list = []
        # keeps track of moves to detect if agents are swapping places
        swaps = {}
        # tracks which boxes are used
        box_actions = defaultdict(int)

        for i, a in enumerate(actions):
            res = None
            if a is not None:
                agent_dir = (a.agent_dir.d_row, a.agent_dir.d_col)
                if a.action_type is ActionType.Move:
                    res = self.move(i, agent_dir)
                elif a.action_type is ActionType.Push:
                    res = self.push(i, agent_dir, (a.box_dir.d_row, a.box_dir.d_col))
                elif a.action_type is ActionType.Pull:
                    res = self.pull(i, agent_dir, (a.box_dir.d_row, a.box_dir.d_col))

            action_list.append(res)
            if res is None:
                continue
            if not res:
                return None

            frees = res[0]
            occupies = res[1]
            agent_from = res[2]
            agent_to = res[3]
            box_id = res[4]

            if frees not in occupation_dict:
                # if we are freeing something there must be something there
                occupation_dict[frees] = 1
            if occupies not in occupation_dict:
                x,y = occupies
                if self.is_free(x, y):
                    occupation_dict[occupies] = 0
                else:
                    return None

            occupation_dict[occupies] += 1

            if box_id is not None:
                box_actions[box_id] += 1
                # more than one agent are trying to move the same box
                if box_actions[box_id] > 1:
                    return None
            # else:
            #     if agent_to in swaps:
            #         if swaps[agent_to] == agent_from:
            #             return None
            #     swaps[agent_from] = agent_to

        # if more than one item occupy the same space the multi action has failed
        for key in occupation_dict.keys():
            if occupation_dict[key] > 1:
                return None

        child = self.copy()
        # finally resolve the resulting state
        for agent_id, action in enumerate(action_list):
            if action is None:
                continue
            _, _, agent_from, agent_to, box_id, box_from, box_to = action
            child.perform_action(agent_id, agent_from, agent_to, box_id, box_from, box_to)
        #child.parent = self
        #child.parent_action = actions
        return child

    def get_StateSA(self, agentID, ignore_immovable=False):
        pos = self.agent_positions[agentID]
        color = self.agent_colors[agentID]

        boxes = []
        goals = []
        extra_walls = []

        #maze: List[List[int]] = None
        #boxes: List[Tuple[int, Tuple[int, int]]] = None,
        #goals: List[Tuple[int, Tuple[int, int]]] = None,
        #agent: Tuple[int, int] = None

        type_dict = {}

        for i, t in enumerate(self.box_types):
            if self.box_colors[i] == color:
                boxes.append((t, self.box_positions[i]))
                type_dict[t] = True
            elif not ignore_immovable:
                extra_walls.append(self.box_positions[i])

        for i, t in enumerate(self.goal_types):
            if t in type_dict:
                goals.append((t, self.goal_positions[i]))

        if not ignore_immovable:
            for pos in self.agent_positions:
                if self.agent_by_cords[pos] != agentID:
                    extra_walls.append(pos)

        if len(extra_walls) == 0:
            return StateSA(self.maze, boxes, goals, self.agent_positions[agentID])

        maze = [[self.maze[i][j] for i in range(self.rows)] for j in range(self.cols)]
        for i, j in extra_walls:
            maze[i][j] = False

        return StateSA(maze, boxes, goals, self.agent_positions[agentID])

    def get_HTN_StateSA(self,agentID,agt_tasks,ignore_immovable=False):
        pos = self.agent_positions[agentID]
        boxes = []
        goals = []
        extra_walls = []

        for t in agt_tasks:
            goals.append((self.goal_types[t[0]], self.goal_positions[t[0]]))
            boxes.append((self.box_types[t[1]], self.box_positions[t[1]]))
        if not ignore_immovable:
            for i in range(len(self.box_colors)):
                if i not in boxes:
                    extra_walls.append(self.box_positions[i])

            for pos in self.agent_positions:
                if self.agent_by_cords[pos] != agentID:
                    extra_walls.append(pos)

        print("boxes for agent:{}".format(agentID),file= sys.stderr,flush=True)
        print(boxes,file= sys.stderr, flush=True)

        print("goals for agent:{}".format(agentID),file= sys.stderr,flush=True)
        print(goals,file= sys.stderr, flush=True)

        print("extra walls:{}".format(extra_walls),file= sys.stderr,flush=True)
        print(extra_walls,file= sys.stderr, flush=True)
        
        if len(extra_walls) == 0:
            return StateSA(self.maze, boxes, goals, self.agent_positions[agentID])

        maze = [[self.maze[i][j] for i in range(self.rows)] for j in range(self.cols)]
        #print(maze,file=sys.stderr,flush=True)
        for i, j in extra_walls:
            maze[i][j] = False

        return StateSA(maze, boxes, goals, self.agent_positions[agentID])

    def get_greedy_StateSA(self, agentID, agt_tasks, ignore_immovable=False):
        pos = self.agent_positions[agentID]
        color = self.agent_colors[agentID]

        boxes = []
        goals = []
        extra_walls = []

        #maze: List[List[int]] = None
        #boxes: List[Tuple[int, Tuple[int, int]]] = None,
        #goals: List[Tuple[int, Tuple[int, int]]] = None,
        #agent: Tuple[int, int] = None

        for bx in agt_tasks[0]:
            boxes.append((self.box_types[bx], self.box_positions[bx]))

        for gs in agt_tasks[1]:
            goals.append((self.goal_types[gs], self.goal_positions[gs]))

        if not ignore_immovable:

            for i in range(len(self.box_colors)):
                if i not in boxes:
                    extra_walls.append(self.box_positions[i])

            for pos in self.agent_positions:
                if self.agent_by_cords[pos] != agentID:
                    extra_walls.append(pos)

        print("boxes for agent:{}".format(agentID),file= sys.stderr,flush=True)
        print(boxes,file= sys.stderr, flush=True)

        print("goals for agent:{}".format(agentID),file= sys.stderr,flush=True)
        print(goals,file= sys.stderr, flush=True)

        print("extra walls:{}".format(extra_walls),file= sys.stderr,flush=True)
        print(extra_walls,file= sys.stderr, flush=True)

        if len(extra_walls) == 0:
            return StateSA(self.maze, boxes, goals, self.agent_positions[agentID])

        maze = [[self.maze[i][j] for i in range(self.rows)] for j in range(self.cols)]
        for i, j in extra_walls:
            maze[i][j] = False

        return StateSA(maze, boxes, goals, self.agent_positions[agentID])


    def __repr__(self):
        lines = []
        chars = "abcdefghijklmnopqrstuvwxyz"
        for row in range(self.rows):
            line = []
            for col in range(self.cols):
                pos = (row,col)
                agent = self.agent_by_cords.get(pos, None)
                box   = self.box_by_cords.get(pos, None)
                goal  = self.goal_by_cords.get(pos, None)
                wall  = ' ' if self.maze[row][col] else '+'
                if agent is not None:
                    line.append(str(agent))
                elif box is not None:
                    if isinstance(self.box_types[box], int):
                        line.append(chars[self.box_types[box]].upper())
                    else:
                        line.append(self.box_types[box].upper())
                elif goal is not None:
                    if isinstance(self.goal_types[goal], int):
                        line.append(chars[self.goal_types[goal]].lower())
                    else:
                        line.append(self.goal_types[goal].lower())
                else:
                    line.append(wall)
            lines.append("".join(line))
        x = "\n".join(lines)
        return x


if __name__ == '__main__':
    from test_state import *

    fail = False
    if not test_swap():
        print("Test swap failed!!")
        fail = True
    if not test_move():
        print("Move test failed!!")
        fail = True
    if not test_push_pull():
        print("push test failed!!")
        fail = True
    if not test_getStateSA():
        print("failed get StateSA")
        fail = True
    if not fail:
        print("All tests passed")


    # maze = [
    #     [False, False, False,False,False,False],
    #     [False, True,  True,True,True,False],
    #     [False, True,  True,True,True,False],
    #     [False, True,  True,True,True,False],
    #     [False, False,  False,False,False,False],
    # ]
    # boxes = []
    # agent = [((2,2),0), ((2,3),0)]
    # goals = []
    #
    # ME = Action(ActionType.Move, Dir.E, None)
    # MS = Action(ActionType.Move, Dir.S, None)
    # MW = Action(ActionType.Move, Dir.W, None)
    # MN = Action(ActionType.Move, Dir.N, None)
    # PSN = Action(ActionType.Pull, Dir.N, Dir.S)
    # PNW = Action(ActionType.Push, Dir.N, Dir.W)
    #
    # initial_state = StateMA(maze, boxes, goals, agent)
    # after_move = initial_state.get_child([ME, MW])
    # print(str(initial_state))
    # print(str(after_move))
