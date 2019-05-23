import random

def print_rect(alpha, height=1, width_mult=1):
    li = list(alpha)
    for i in range(height):
        line = ""
        for j in range(width_mult):
            random.shuffle(li)
            line += "".join(li)
        print(line)


print_rect("IJKLMNOP", 8, 1)