grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    
    # define array with the same size all the chekc marks are not there
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid[1]))]
    closed[init[0]][init[1]] = 1
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid[1]))]
    action= [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    
    open = [[g, x, y]] # initial open list
    
    found = False # flag that is set when search is completed
    resign = False # flag set if we can't find expand   
    count = 0 # counts the expansion
    
    print('initial open list:')
    for i in range(len(open)):
        print('   ', open[i])
    print('----')

    while found is False and resign is False:
        
        # check if we still have elements on the open list
        if len(open) == 0:
            resign = True
            print('fail: no more elements on the open list')
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
            x = next[1]
            y = next[2]
            g = next[0]
            expand[x][y] = count
            count += 1
            
            # check if search is finished
            
            if x == goal[0] and y == goal[1]:
                found = True
                print('search success: ', next)
                
            else:
                # YOU always expand the one with the smalles G-value
                # expand winning element and add to new open list
                # check up,down,left,right
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            print('open before append',open)
                            g2 = g + cost
                            print('append list item',[g2, x2, y2])
                            open.append([g2, x2, y2])
                            print('open after append',open)
                            # check the coordinate x2 y2 so that I never expand it again 
                            closed[x2][y2] = 1  
                            # path
                            action[x2][y2] = i # memorize the action it took to get there
            print('')
            
    print('')
    print('EXPANSION COUNT (order in which "next" x,y,g were chosen')                        
    for i in range(len(expand)):
        print(expand[i])

    print('')
    print('ACTION (up,left,down,right)')                        
    for i in range(len(action)):
        print(action[i])

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
    print('')
    print('PATH')                        
    for i in range(len(policy)):
        print(policy[i])
    return


search(grid,init,goal,cost)