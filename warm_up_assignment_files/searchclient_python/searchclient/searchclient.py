import argparse
import re
import sys

import memory
from state import State
from strategy import StrategyBFS, StrategyDFS, StrategyBestFirst
from heuristic import AStar, WAStar, Greedy


class SearchClient:
    def __init__(self, server_messages):
        self.initial_state = None
        
        colors_re = re.compile(r'^[a-z]+:\s*[0-9A-Z](\s*,\s*[0-9A-Z])*\s*$')
        try:
            # Read lines for colors. There should be none of these in warmup levels.
            line = server_messages.readline().rstrip()
            if colors_re.fullmatch(line) is not None:
                print('Error, client does not support colors.', file=sys.stderr, flush=True)
                sys.exit(1)
            
            # Read lines for level.
            self.initial_state = State()
            row = 0
            while line:
                for col, char in enumerate(line):
                    if char == '+': self.initial_state.walls[row][col] = True
                    elif char in "0123456789":
                        if self.initial_state.agent_row is not None:
                            print('Error, encountered a second agent (client only supports one agent).', file=sys.stderr, flush=True)
                            sys.exit(1)
                        self.initial_state.agent_row = row
                        self.initial_state.agent_col = col
                    elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ": self.initial_state.boxes[row][col] = char
                    elif char in "abcdefghijklmnopqrstuvwxyz": self.initial_state.goals[row][col] = char
                    elif char == ' ':
                        # Free cell.
                        pass
                    else:
                        print('Error, read invalid level character: {}'.format(char), file=sys.stderr, flush=True)
                        sys.exit(1)
                row += 1
                line = server_messages.readline().rstrip()
        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            sys.exit(1)
    
    def search(self, strategy: 'Strategy') -> '[State, ...]':
        print('Starting search with strategy {}.'.format(strategy), file=sys.stderr, flush=True)
        strategy.add_to_frontier(self.initial_state)
        
        iterations = 0
        while True:
            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0
            
            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None
            
            if strategy.frontier_empty():
                return None
            
            leaf = strategy.get_and_remove_leaf()
            
            if leaf.is_goal_state():
                return leaf.extract_plan()
            
            strategy.add_to_explored(leaf)
            for child_state in leaf.get_children(): # The list of expanded states is shuffled randomly; see state.py.
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            
            iterations += 1


def main(strategy_str: 'str'):
    # Read server messages from stdin.
    server_messages = sys.stdin
    
    # Use stderr to print to console through server.
    print('SearchClient initializing. I am sending this using the error output stream.', file=sys.stderr, flush=True)
    
    # Read level and create the initial state of the problem.
    client = SearchClient(server_messages);

    strategy = None
    if strategy_str == 'bfs':
        strategy = StrategyBFS()
    elif strategy_str == 'dfs':
        strategy = StrategyDFS()
    elif strategy_str == 'astar':
        strategy = StrategyBestFirst(AStar(client.initial_state))
    elif strategy_str == 'wastar':
        strategy = StrategyBestFirst(WAStar(client.initial_state, 5))
    elif strategy_str == 'greedy':
        strategy = StrategyBestFirst(Greedy(client.initial_state))
    else:
        # Default to BFS strategy.
        strategy = StrategyBFS()
        print('Defaulting to BFS search. Use arguments -bfs, -dfs, -astar, -wastar, or -greedy to set the search strategy.', file=sys.stderr, flush=True)
    
    solution = client.search(strategy)
    if solution is None:
        print(strategy.search_status(), file=sys.stderr, flush=True)
        print('Unable to solve level.', file=sys.stderr, flush=True)
        sys.exit(0)
    else:
        print('\nSummary for {}.'.format(strategy), file=sys.stderr, flush=True)
        print('Found solution of length {}.'.format(len(solution)), file=sys.stderr, flush=True)
        print('{}.'.format(strategy.search_status()), file=sys.stderr, flush=True)
        
        for state in solution:
            print(state.action, flush=True)
            response = server_messages.readline().rstrip()
            if 'false' in response:
                print('Server responsed with "{}" to the action "{}" applied in:\n{}\n'.format(response, state.action, state), file=sys.stderr, flush=True)
                break


if __name__ == '__main__':
    # Program arguments.
    parser = argparse.ArgumentParser(description='Simple client based on state-space graph search.')
    parser.add_argument('--max-memory', metavar='<MB>', type=float, default=2048.0, help='The maximum memory usage allowed in MB (soft limit, default 2048).')
    
    strategy_group = parser.add_mutually_exclusive_group()
    strategy_group.add_argument('-bfs', action='store_const', dest='strategy', const='bfs', help='Use the BFS strategy.')
    strategy_group.add_argument('-dfs', action='store_const', dest='strategy', const='dfs', help='Use the DFS strategy.')
    strategy_group.add_argument('-astar', action='store_const', dest='strategy', const='astar', help='Use the A* strategy.')
    strategy_group.add_argument('-wastar', action='store_const', dest='strategy', const='wastar', help='Use the WA* strategy.')
    strategy_group.add_argument('-greedy', action='store_const', dest='strategy', const='greedy', help='Use the Greedy strategy.')
    
    args = parser.parse_args()
    
    # Set max memory usage allowed (soft limit).
    memory.max_usage = args.max_memory
    
    # Run client.
    main(args.strategy)

