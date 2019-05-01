"""
import test_utilities as tu

agt0 = tu.agent(0,"red")
agt1 = tu.agent(1,"blue")
level = [
    [False, False,False,False],
    [False, agt0,agt1,False],
    [False, False,False,False]
]
s = tu.make_state(level)
action = [tu.move(tu.east), tu.move(tu.west)]
x = s.get_child(action)

y = 123
"""