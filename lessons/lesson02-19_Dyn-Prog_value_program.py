# ----------
# User Instructions:
# 
# The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

init = [0, 0]

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]

goal = [len(grid)-1, len(grid[0])-1]

cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    
    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
 
    change = True
    
    while change:
        
        change = False # will change to True if I actually change something
        
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                
                # first check if the grid cell i'm considering is the goal
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        change = True
                
                # if it's not the goal cell, this is the whole update function
                elif grid[x][y] == 0:
                    print('elif')
                    for a in range(len(delta)):
                        # next state x2,y2
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]
                        
                        # test if state is inside the grid and it is a navigable grid cell 0
                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            
                            
                            v2 = value[x2][y2] + cost
                            print('elif-if','v2=',v2,'=',value[x2][y2],'+',cost)
                            
                            if v2 < value[x][y]:
                                print('elif-if-if')
                                change = True
                                value[x][y] = v2
                                print('value[x][y]=',value[x][y])
    
    for i in range(len(value)):
        print(value[i]) 
    
    return value 

compute_value(grid,goal,cost)