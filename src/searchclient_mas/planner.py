"""
This file implements a general purpose best-first planner.

--------------HOW TO INITIALIZE IT -------------
An instance of the planner is created using

planner = Planner(s)

where s is the initial state in the planning process.

The planner needs five functions/methods to work properly.
These functions can either be implemented as methods of the state 's',
or provided as optional arguments to the constructor. They are:

get_children : state -> iterable collection of states.
This function takes a state and returns all its neighbours in the state space

is_goal_state : state -> bool
This function returns true if the provided state is a goal state

extract_plan : state -> whatever result you want
This function generates some sort of plan from the goal state.
This is what the planner returns, and the planner itself doesn't care
about how the extracted plan looks

heuristic : state -> int
This function produces a heuristic value for the provided state.
Technically, it could map each state to anything that is comparable,
but integers are fast, so lets stick to that.

get_g_value : state -> int
This funciton gives the distance from the initial state of the search to this state


The functions provided as arguments take precedence over method implementations in 's'.
This means that it is possible to implement all methods in 's',
but still provide a custom heuristic function to the planner in the following way:

planner = Planner(s, heuristic=custom_heuristic_function)


---------------- HOW TO USE IT ----------------------------
The planner supports three different ways of searching the state space:
expand_one_state - picks one state from the frontier and expands it. It is more or less useless, and used only as a sub procedure

expand_n_states  - this method takes an integer 'n' as argument and repeats expand_one_state 'n' times or until a successful plan is found.
This is useful if you want to search for a while, but stop if the search takes too long

If a plan is found during the execution of any of the above procedues, it is stored in the attribute 'plan' of the planner.
I.e., get it using (some_planner.plan). The attribute 'plan' is None if no plan is found

The last way of finding a plan is:

make_plan - This method starts searching and stops when a plan is found or the state space is exhausted.
It returns None if it couldn't find a plan, or the plan itself otherwise
"""
import heapq
import sys
from time import perf_counter

def default_heuristic(state):
    return state.heuristic()

def default_get_g_value(state):
    return state.get_g_value()

def default_is_goal_state(state):
    return state.is_goal_state()

def default_extract_plan(state):
    return state.extract_plan()

def default_get_children(state):
    return state.get_children()

class Planner(object):
    def __init__(self,initial_state,get_children = None,is_goal_state = None,extract_plan = None, heuristic = None, g_value = None, cutoff_solution_length = None, print_status = True):
        #Setting the functions used to explore the state space
        #Use implementaitons in state unless new functions are provided
        self.get_children = get_children if get_children else default_get_children
        self.is_goal_state = is_goal_state if is_goal_state else default_is_goal_state
        self.extract_plan = extract_plan if extract_plan else default_extract_plan
        self.heuristic = heuristic if heuristic else default_heuristic
        self.g_value = g_value if g_value else default_get_g_value

        #Adding the initial state to the frontier
        self.frontier = []
        heapq.heapify(self.frontier)
        firstEntry = (self.heuristic(initial_state), initial_state)
        heapq.heappush(self.frontier, firstEntry)

        #Initialize remaining variables
        self.expanded_set = set()
        self.plan = None

        #Replace the cutoff with a suitably large number if there is no cutoff
        self.cutoff_solution_length = cutoff_solution_length if cutoff_solution_length else 2000000000

        #output planner status
        self.print_status = print_status
        self.start_time = perf_counter()
        self.times_printed = []

    def expand_one_state(self):
        #TODO: Fix this: it's not very good. What if there is no solution and the state space is exhausted?
        assert len(self.frontier) > 0, "state space exhausted in planner"

        #Extract the state with minimum heuristic value from the frontier
        result = heapq.heappop(self.frontier)
        state = result[1]

        #Find the plan if state is goal
        if self.is_goal_state(state):
            #if hasattr(state, 'game_state'):
            #    print("--------------------------FOUND THE GOAL-----------------",file=sys.stderr)
            #    print(state.game_state,file=sys.stderr)
            self.plan = self.extract_plan(state)
            return

        #Add the state to the expanded set
        self.expanded_set.add(state)

        #Get the unexpanded neighbours of the state
        children = self.get_children(state)

        #Filter out expanded states and states that are past the cutoff for solution length
        children = [s for s in children if s not in self.expanded_set and self.g_value(s) < self.cutoff_solution_length]

        #Calculate their heuristic value
        children = [(self.heuristic(s),s) for s in children]

        #Add them to the frontier
        for entry in children:
            heapq.heappush(self.frontier, entry)

        # will print the current search status roughly every 10 seconds
        if self.print_status:
            if not int('%.f' % self.used_time()) % 10 and int('%.f' % self.used_time()) not in self.times_printed:
                self.times_printed.append(int('%.f' % self.used_time()))
                print(self.search_status(), file=sys.stderr, flush=True)

    #Expands at most n more states from the frontier.
    #Returns the plan if it is found, otherwise returns None
    def expand_n_states(self, n):
        for i in range(n):
            if self.plan:
                return self.plan
            self.expand_one_state()
        return self.plan

    #finds a plan to the problem. If there is no goal state, returns None
    def make_plan(self):
        while(len(self.frontier) > 0 and not self.plan):
            self.expand_one_state()
        return self.plan

    def used_time(self):
        return perf_counter()-self.start_time

    def search_status(self):
        return "expanded: {}, frontier: {}, generated: {}, time: {:3.2f} ".format(len(self.expanded_set), len(self.frontier), len(self.expanded_set)+len(self.frontier), self.used_time())
