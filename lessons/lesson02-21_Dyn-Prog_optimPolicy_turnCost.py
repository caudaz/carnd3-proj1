# Computes and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------

def optimum_policy2D(grid,init,goal,cost):
    
    # 3d grid (x, y, and 4 orientations)
    value  =  [[[999 for row in range(len(grid[0]))] for col in range(len(grid))],
               [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
               [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
               [[999 for row in range(len(grid[0]))] for col in range(len(grid))]]
    # 3d grid (x, y, and 4 orientations)    
    policy =  [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
               [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
               [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
               [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]
    # 2d grid (x, y)    
    policy2D = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]    
    
    change = True    
    while change:       
        change = False        
        # go thru all grids and calculate values (all X's, Y's and 4 orientations)
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(4):
                    # if goal location found update value
                    if goal[0] == x and goal[1] == y:
                        if value[orientation][x][y] > 0:
                            # mark change!
                            change = True
                            value[orientation][x][y] = 0
                            # mark location with an asterisk
                            policy[orientation][x][y] = '*'
                    # navigable space in grid (not an obstacle)
                    elif grid[x][y] == 0:
                        # calculate the 3 ways to propagate value
                        for i in range(3):
                            o2 = (orientation + action[i]) % 4
                            x2 = x + forward[o2][0]
                            y2 = y + forward[o2][1]
                            # if we arrive at a valid grid cell and it's inside the grid
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                                # add the cost of the corresponding action
                                v2 = value[o2][x2][y2] + cost[i]
                                # if it improves over the existing value
                                if v2 < value[orientation][x][y]:
                                    # set the value to be the new value
                                    value[orientation][x][y] = v2
                                    # memorize the action name
                                    policy[orientation][x][y] = action_name[i]
                                    # mark change!
                                    change = True

    x = init[0]
    y = init[1]
    orientation = init[2]

    policy2D[x][y] = policy[orientation][x][y]
    while policy[orientation][x][y] != '*':
        # going straight
        if policy[orientation][x][y] == '#':
            o2 = orientation
        # turn right
        elif policy[orientation][x][y] == 'R':
            o2 = (orientation - 1) % 4
        # turn left
        elif policy[orientation][x][y] == 'L':
            o2 = (orientation + 1) % 4   
        # apply forward motion and update orientation
        x = x + forward[o2][0]
        y = y + forward[o2][1]
        orientation = o2
        policy2D[x][y] = policy[orientation][x][y]
        
    for i in range(len(policy2D)):
        print(policy2D[i])

        
    return policy2D


optimum_policy2D(grid,init,goal,cost)