# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]
heuristic_old = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h

    open = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            # remove nodes from list
            print('open before sort',open)
            open.sort()
            print('open after sort',open)
            open.reverse()
            print('open after reverse',open)
            next = open.pop() # finds the one with the smalles g-value to be expanded
            print('open after next=open.pop()',open)
            print('next',next)
            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            print('open before append',open)
                            g2 = g + cost
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2
                            print('append list item', [f2, g2, h2, x2, y2])
                            open.append([f2, g2, h2, x2, y2])
                            print('open after append',open)
                            # close x2 y2 so that it cannot be expanded/search again
                            closed[x2][y2] = 1
                            # path
                            action[x2][y2] = i # memorize the action it took to get there
            print('')
               
    print('EXPANSION COUNT')                        
    for i in range(len(expand)):
        print(expand[i])        
    print('')
    
    print('ACTION (up,left,down,right)')                        
    for i in range(len(action)):
        print(action[i])
    print('')
    
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*' # goal location
    while x != init[0] or y != init[1]:
         x2 = x - delta[action[x][y]][0]
         y2 = y - delta[action[x][y]][1]
         policy[x2][y2] = delta_name[action[x][y]]
         x = x2
         y = y2                           
    print('PATH')                        
    for i in range(len(policy)):
        print(policy[i])

    return

search(grid,init,goal,cost,heuristic)