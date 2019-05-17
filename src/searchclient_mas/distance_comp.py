from state import StateMA
from queue import Queue
from goal_analyzer import get_neighbours
import test_utilities as tu

def manhattan_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

class Node:
    def __init__(self, pos, value):
        self.pos = pos
        self.value = value


class DistanceComputer:
    def __init__(self, state: StateMA):
        self.maze = state.maze
        self.distdict = {}
        self.seendict = {}
        self.queue_dict = {}

    def dist(self, src, dst):
        #return manhattan_dist(src,dst)
        if (src, dst) in self.distdict:
            return self.distdict[(src,dst)]

        if src == dst:
            return 0

        if src not in self.seendict:
            seen = set()
            seen.add(src)
            q = Queue()
            q.put(Node(src, 0))
            self.seendict[src] = seen
            self.queue_dict[src] = q

        return self.compute_bfs(src, dst, self.seendict[src], self.queue_dict[src])



    def compute_bfs(self, src, dst, seen:set, queue:Queue):

        while not queue.empty():
            curr = queue.get()
            pos = curr.pos
            val = curr.value

            self.distdict[(src, pos)] = val
            self.distdict[(pos, src)] = val

            neighbors = get_neighbours(pos)

            for n in neighbors:
                if n not in seen and self.maze[n[0]][n[1]]:
                    seen.add(n)
                    queue.put(Node(n, val+1))

            if pos == dst:
                return self.distdict[(src, pos)]

        assert False, "given positions dont seem to be in same component"




if __name__ == '__main__':

    maze = tu.create_maze(10)
    for i in range(1, 9):
        maze[1][i] = tu.box('a', 'blue')
        maze[2][i] = tu.box('b', 'green')


    for i in range(3, 9):
        maze[6][i] = False

    maze[2][2] = tu.box('b', 'blue')
    maze[8][5] = tu.goal('a')

    maze[3][5] = tu.agent(0, 'blue')
    maze[4][5] = tu.agent(1, 'green')

    maze[8][8] = tu.box('s', 'green')
    maze[7][7] = tu.box('d', 'green')

    state = tu.make_state(maze)
    print(state)
    dst = DistanceComputer(state)

    print(dst.dist((8,8),(7,7)))
    print(dst.dist((8,8),(7,7)))
    print(dst.dist((8,8),(3,5)))
    print(dst.dist((7,7),(3,5)))