from goal_analyzer import GoalAnalyzer
from state import StateMA

class ParallelPlanner:
    def __init__(self, state: StateMA):
        self.state = state
        self.goal_analyzer = GoalAnalyzer(state)
        self.goal_order = self.goal_analyzer.compute_goal_order_plan()
        self.immovable_boxes = {}
        self.current_plan = []

    def find_box_and_agent_for_goal(self, goal_id):
        # TODO: do this in a smart way, not at random
        # TODO: have this function yield? in case found match is not usable we don't wanna restart the process
        # TODO: what if this is agent goal?
        g_type = self.state.goal_types[goal_id]
        viable_boxes = []

        for b_type, i in enumerate(self.state.box_types):
            if i in self.immovable_boxes:
                continue
            if g_type == b_type:
                viable_boxes.append(i)

        for a_color, i in enumerate(self.state.agent_colors):
            for b in viable_boxes:
                if self.state.box_colors[b] == self.state.agent_colors:
                    return b, i

    def find_path_to_box(self, agent_id, box_id):
        # TODO: implement
        pass

    def move_to_storage(self, id, is_box=True):
        # TODO: implement
        # TODO: need some notion of storage spaces
        pass

    def move_box_to_goal(self, agent_id, box_id, goal_id):
        # TODO: implement
        pass

    def compute_plan(self):
        # TODO : change this to work for agent goals
        for goal in self.goal_order:
            #TODO: move required stuff out of rooms getting blocked when goal is completed
            box, agent = self.find_box_and_agent_for_goal(goal)

            path = self.find_path_to_box(agent, box)
            # TODO: for item on path move to storage

            path = self.move_box_to_goal(agent, box, goal)
            # TODO: for item on path move to storage


