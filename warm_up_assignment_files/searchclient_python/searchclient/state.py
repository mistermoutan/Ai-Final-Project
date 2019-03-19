import random

from action import ALL_ACTIONS, ActionType


class State:
    _RNG = random.Random(1)
    MAX_ROW = 70
    MAX_COL = 70
    
    def __init__(self, copy: 'State' = None):
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
        if copy is None:
            self.agent_row = None
            self.agent_col = None
            
            self.walls = [[False for _ in range(State.MAX_COL)] for _ in range(State.MAX_ROW)]
            self.boxes = [[None for _ in range(State.MAX_COL)] for _ in range(State.MAX_ROW)]
            self.goals = [[None for _ in range(State.MAX_COL)] for _ in range(State.MAX_ROW)]
            
            self.parent = None
            self.action = None
            
            self.g = 0
        else:
            self.agent_row = copy.agent_row
            self.agent_col = copy.agent_col
            
            self.walls = [row[:] for row in copy.walls]
            self.boxes = [row[:] for row in copy.boxes]
            self.goals = [row[:] for row in copy.goals]
            
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
                        child.boxes[self.agent_row][self.agent_col] = self.boxes[box_row][box_col]
                        child.boxes[box_row][box_col] = None
                        child.parent = self
                        child.action = action
                        child.g += 1
                        children.append(child)
        
        State._RNG.shuffle(children)
        return children
    
    def is_initial_state(self) -> 'bool':
        return self.parent is None
    
    def is_goal_state(self) -> 'bool':
        for row in range(State.MAX_ROW):
            for col in range(State.MAX_COL):
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
                if self.boxes[row][col] is not None: line.append(self.boxes[row][col])
                elif self.goals[row][col] is not None: line.append(self.goals[row][col])
                elif self.walls[row][col] is not None: line.append('+')
                elif self.agent_row == row and self.agent_col == col: line.append('0')
                else: line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)

