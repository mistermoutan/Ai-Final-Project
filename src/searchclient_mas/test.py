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
def get_room(val,_dict): 
    for room, value in _dict.items(): 
        print(value)
        if val in value: 
            return room
    return None
  
goals_per_room={0: {(3, 10), (6, 8), (6, 1)}}
print(get_room((3,10),goals_per_room))
