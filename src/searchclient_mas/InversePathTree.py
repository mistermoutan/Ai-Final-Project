from typing import Tuple

class Node:
    def __init__(self, vertex):
        self.vertex = vertex
        self.next = None

    def __repr__(self):
        return str(self.vertex)


class InversePathTree:
    def __init__(self, root):
        self.node_dict = {root: Node(root)}
        self.queue = []


    def add_node(self, vertex: Tuple[int, int]):
        if vertex is None:
            return False

        nxt = None
        if vertex in self.node_dict:
            nxt = self.node_dict[vertex]

            while self.queue:
                curr = Node(self.queue.pop())
                curr.next = nxt
                nxt = curr
                self.node_dict[curr.vertex] = curr
            return False
        else:
            self.queue.append(vertex)
            return True

    def get_path(self, vertex):
        node = self.node_dict[vertex]
        while node.next is not None:
            yield node.vertex
            node = node.next
        yield node.vertex


if __name__ == '__main__':
    path1 = reversed([(1, i) for i in range(6)])
    path2 = reversed([(1+i//3, i) for i in range(6)])

    paths = InversePathTree((1, 0))

    for i in path1:
        if not paths.add_node(i):
            break

    for i in path2:
        if not paths.add_node(i):
            break

    for i in paths.get_path((1, 5)):
        print(i, end="")
    print("\n")

    for i in paths.get_path((2, 4)):
        print(i, end="")
    print("\n")

    for i in paths.get_path((1, 0)):
        print(i, end="")
    print("\n")