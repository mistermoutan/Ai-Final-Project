from action import north,south,east,west,Dir

#Returns a new list containing every consecutive pairs of elements from the list
#I.e. given [x1,x2,x3....xn-1], returns [(x1,x2),(x2,x3),...,(xn-2,xn-1)]
def pairwise(list):
    return [(list[i], list[i+1]) for i in range(len(list) - 1)]

#Given a position and a direction, returns the neighbour of that position in that direction
def neighbour_in_direction(position,direction):
    x,y = position
    return (x + direction.d_row, y + direction.d_col)

#Returns true if pos1 and pos2 are adjacent, false otherwise
def are_adjacent(pos1,pos2):
    x1,y1 = pos1
    x2,y2 = pos2
    delta = abs(x1-x2) + abs(y1-y2)
    return delta == 1

delta_to_direction_map = {
    (1,0):  south,
    (-1,0): north,
    (0,1):  east,
    (0,-1): west
}
def direction_between_adjacent_positions(pos1,pos2):
    assert are_adjacent(pos1,pos2), "pos1 and pos2 are not adjacent"
    delta_x = pos2[0]-pos1[0]
    delta_y = pos2[1]-pos1[1]
    
    return delta_to_direction_map.get( (delta_x, delta_y) )