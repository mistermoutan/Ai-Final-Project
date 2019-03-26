import random
from collections import defaultdict
from action import ALL_ACTIONS, ActionType
from typing import List, Tuple
import sys


class State:
    # _RNG = random.Random(1)
    MAX_ROW = 70
    MAX_COL = 70

    def __init__(self, copy: 'State' = None, rows=None, cols=None):
        '''
        If copy is None: Creates an empty State.
        If copy is not None: Creates a copy of the copy state.

        The lists walls, boxes, and goals are indexed from top-left of the level, row-major order (row, col).
               Col 0  Col 1  Col 2  Col 3
        Row 0: (0,0)  (0,1)  (0,2)  (0,3)  ...
        Row 1: (1,0)  (1,1)  (1,2)  (1,3)  ...
        Row 2: (2,0)  (2,1)  (2,2)  (2,3)  ...
        ...

        For example, self.walls is a list of size [MAX_ROW][MAX_COL] and
        self.walls[2][7] is True if there is a wall at row 2, column 7 in this state.

        Note: The state should be considered immutable after it has been hashed, e.g. added to a dictionary!
        '''
        self._hash = None
        self.rows = rows
        self.cols = cols
        if copy is None:
            self.agent_row = None
            self.agent_col = None

            self.walls = [[False for _ in range(self.cols)] for _ in range(self.rows)]
            self.boxes = [[None for _ in range(self.cols)] for _ in range(self.rows)]
            self.goals = [[None for _ in range(self.cols)] for _ in range(self.rows)]
            self.goal_dict = defaultdict(list)

            self.parent = None
            self.action = None

            self.g = 0
        else:
            self.rows = copy.rows
            self.cols = copy.cols

            self.agent_row = copy.agent_row
            self.agent_col = copy.agent_col

            self.walls = copy.walls
            self.boxes = copy.boxes
            self.goals = copy.goals
            self.goal_dict = copy.goal_dict

            self.parent = copy.parent
            self.action = copy.action

            self.g = copy.g

    def get_children(self) -> '[State, ...]':
        '''
        Returns a list of child states attained from applying every applicable action in the current state.
        The order of the actions is random.
        '''
        children = []
        for action in ALL_ACTIONS:
            # Determine if action is applicable.
            new_agent_row = self.agent_row + action.agent_dir.d_row
            new_agent_col = self.agent_col + action.agent_dir.d_col

            if action.action_type is ActionType.Move:
                if self.is_free(new_agent_row, new_agent_col):
                    child = State(self)
                    child.agent_row = new_agent_row
                    child.agent_col = new_agent_col
                    child.parent = self
                    child.action = action
                    child.g += 1
                    children.append(child)
            elif action.action_type is ActionType.Push:
                if self.box_at(new_agent_row, new_agent_col):
                    new_box_row = new_agent_row + action.box_dir.d_row
                    new_box_col = new_agent_col + action.box_dir.d_col
                    if self.is_free(new_box_row, new_box_col):
                        child = State(self)
                        child.agent_row = new_agent_row
                        child.agent_col = new_agent_col
                        child.boxes = [b for b in child.boxes]
                        child.boxes[new_box_row] = [b for b in child.boxes[new_box_row]]
                        child.boxes[new_agent_row] = [b for b in child.boxes[new_agent_row]]
                        child.boxes[new_box_row][new_box_col] = self.boxes[new_agent_row][new_agent_col]
                        child.boxes[new_agent_row][new_agent_col] = None
                        child.parent = self
                        child.action = action
                        child.g += 1
                        children.append(child)
            elif action.action_type is ActionType.Pull:
                if self.is_free(new_agent_row, new_agent_col):
                    box_row = self.agent_row + action.box_dir.d_row
                    box_col = self.agent_col + action.box_dir.d_col
                    if self.box_at(box_row, box_col):
                        child = State(self)
                        child.agent_row = new_agent_row
                        child.agent_col = new_agent_col
                        child.boxes = [b for b in child.boxes]
                        child.boxes[self.agent_row] = [b for b in child.boxes[self.agent_row]]
                        child.boxes[box_row] = [b for b in child.boxes[box_row]]
                        child.boxes[self.agent_row][self.agent_col] = self.boxes[box_row][box_col]
                        child.boxes[box_row][box_col] = None
                        child.parent = self
                        child.action = action
                        child.g += 1
                        children.append(child)

        # State._RNG.shuffle(children)
        return children

    def is_initial_state(self) -> 'bool':
        return self.parent is None

    def is_goal_state(self) -> 'bool':
        for row in range(self.rows):
            for col in range(self.cols):
                goal = self.goals[row][col]
                box = self.boxes[row][col]
                if goal is not None and (box is None or goal != box.lower()):
                    return False
        return True

    def is_free(self, row: 'int', col: 'int') -> 'bool':
        return not self.walls[row][col] and self.boxes[row][col] is None

    def box_at(self, row: 'int', col: 'int') -> 'bool':
        return self.boxes[row][col] is not None

    def extract_plan(self) -> '[State, ...]':
        plan = []
        state = self
        while not state.is_initial_state():
            plan.append(state)
            state = state.parent
        plan.reverse()
        return plan

    def __hash__(self):
        if self._hash is None:
            prime = 31
            _hash = 1
            _hash = _hash * prime + self.agent_row
            _hash = _hash * prime + self.agent_col
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.boxes))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.goals))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.walls))
            self._hash = _hash
        return self._hash

    def __lt__(self, other):
        return False

    def __eq__(self, other):
        if self is other: return True
        if not isinstance(other, State): return False
        if self.agent_row != other.agent_row: return False
        if self.agent_col != other.agent_col: return False
        if self.boxes != other.boxes: return False
        if self.goals != other.goals: return False
        if self.walls != other.walls: return False
        return True

    def __repr__(self):
        lines = []
        for row in range(State.MAX_ROW):
            line = []
            for col in range(State.MAX_COL):
                if self.boxes[row][col] is not None:
                    line.append(self.boxes[row][col])
                elif self.goals[row][col] is not None:
                    line.append(self.goals[row][col])
                elif self.walls[row][col] is not None:
                    line.append('+')
                elif self.agent_row == row and self.agent_col == col:
                    line.append('0')
                else:
                    line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)


repr_dict = {
    0: 'A',
    1: 'B',
    2: 'C',
    3: 'D',

}


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
        self.box_at_position = {pos: i for i, pos in enumerate(self.box_positions)}

        self.goal_types = [b[0] for b in goals]
        self.goal_positions = [b[1] for b in goals]
        self.goal_at_position = {pos: i for i, pos in enumerate(self.goal_positions)}

        self.agent_row = agent[0]
        self.agent_col = agent[1]

        self.parent = None
        self.g = 0
        self.action = None

        self._hash = None

    def copy(self):
        # shallow copy for more efficient get children, especially if agent is only being moved
        cpy = StateSA()
        cpy.rows = self.rows

        cpy.cols = self.cols
        cpy.maze = self.maze

        cpy.box_types = self.box_types
        cpy.box_positions = self.box_positions
        cpy.box_at_position = self.box_at_position

        cpy.goal_types = self.goal_types
        cpy.goal_positions = self.goal_positions
        cpy.goal_at_position = self.goal_at_position
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
        self.box_at_position = dict(self.box_at_position)

        box_id = self.box_at_position[old_pos]
        self.box_positions[box_id] = new_pos
        del self.box_at_position[old_pos]
        self.box_at_position[new_pos] = box_id

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
            if p in self.box_at_position:
                box_id = self.box_at_position[p]
                if self.goal_types[i] != self.box_types[box_id]:
                    return False
            else:
                # print("failed: empty",(x,y), file=sys.stderr, flush=True)
                return False
        return True

    def is_free(self, row: 'int', col: 'int') -> 'bool':
        return self.maze[row][col] and (row, col) not in self.box_at_position

    def box_at(self, row: 'int', col: 'int') -> 'bool':
        return (row, col) in self.box_at_position

    def extract_plan(self) -> '[StateSA, ...]':
        plan = []
        state = self
        while not state.is_initial_state():
            plan.append(state)
            state = state.parent
        plan.reverse()
        return plan

    def box_hash(self):
        return sum(hash((self.box_types[i], self.box_positions[i])) for i in range(len(self.box_types)))

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
            if pos not in other.box_at_position:
                return False
            else:
                curr_id = self.box_at_position[pos]
                other_id = other.box_at_position[pos]
                if self.box_types[curr_id] != other.box_types[other_id]:
                    return False
        return True

    def __repr__(self):
        lines = []
        for row in range(self.rows):
            line = []
            for col in range(self.cols):
                pos = (row, col)
                if pos in self.box_at_position:
                    line.append(repr_dict[self.box_types[self.box_at_position[pos]]])
                elif self.agent_row == row and self.agent_col == col:
                    line.append('0')
                elif pos in self.goal_at_position:
                    line.append(repr_dict[self.goal_types[self.goal_at_position[pos]]].lower())
                elif not self.maze[row][col]:
                    line.append('+')
                else:
                    line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)

