# Random Planner and Optimal Planner
"""
Created on Thu May 24 23:26:04 2018

@author: sid_1
"""

import math
import random

# ** INPUTS ** 

#world_state = The 2D grid with 0 representing the movement space, 1 representing obstacles 

#  Example:    [[0, 0, 1, 0, 0, 0],
#               [1, 0, 1, 0, 0, 0],
#               [1, 0, 1, 0, 1, 0],
#               [1, 0, 1, 0, 1, 0],
#               [1, 0, 1, 1, 1, 0],
#               [0, 0, 0, 0, 0, 0]]

world_state = [[0, 0, 0, 0, 0, 0],
               [1, 1, 0, 0, 1, 0],
               [1, 0, 0, 0, 0, 0],
               [1, 0, 0, 0, 1, 0],
               [1, 0, 1, 0, 1, 0],
               [0, 0, 0, 0, 0, 0]]

#robot_pose = [row,column] position of goal in world_state
robot_pose = [0, 0]

#goal_pose : [row,column] position of goal in world_state
goal_pose = [4, 5]


# ****Movements *****

move = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

move_sign = ['^', '<', 'v', '>']

# ************************** Random Planner *************************
def RandomSearch(world_state,robot_pose,goal_pose):
   
    # Closed is a 2D array to track the cells with 1 representing cells we have visited already
    closed = [[0 for col in range(len(world_state[0]))] for row in range(len(world_state))]
    
    action = [[-1 for col in range(len(world_state[0]))] for row in range(len(world_state))]
 
    x = robot_pose[0]
    y = robot_pose[1]
    
    # Initialising the source robot cell as visited
    closed[x][y] = 1

    # open is the set of cell positions that we have opened and are under consideration for movement
    open = [[x, y]] 

    found = False  # flag that is set when search is complete and goal is not found
    resign = False # flag set if we can't find expand
    current_step = 0
    max_step_number = 1000
    
    while not found and not resign and current_step < max_step_number:
        if len(open) == 0:
            resign = True
            return "Random Search Failed"
        else:
            random.shuffle(open)
            #next = open[int(random.random()*len(open))]
            next = open.pop()
            x = next[0]
            y = next[1]
            #print(open)
                        
            if x == goal_pose[0] and y == goal_pose[1]:
                found = True
            else:
                #expand the element that comes through and add to the open list
                for i in range(len(move)):
                    x2 = x + move[i][0]
                    y2 = y + move[i][1]
                    if x2 >= 0 and x2 < len(world_state) and y2 >=0 and y2 < len(world_state[0]):
                        if closed[x2][y2] == 0 and world_state[x2][y2] == 0:
                            open.append([x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i # memorising the action it took to reach there, notice that we dont associate with x and y the from state
                            current_step += 1
                            
    policy = [[' ' for col in range(len(world_state[0]))] for row in range(len(world_state))]
    x = goal_pose[0]
    y = goal_pose[1]
    policy[x][y] = '*'
    
    path = []
    path.append([x,y])
   # we go from goal_pose backwards till we find the robot_poseial position
    while x != robot_pose[0] or y!=robot_pose[1]:
        x2 = x - move[action[x][y]][0]
        y2 = y - move[action[x][y]][1]
        policy[x2][y2] = move_sign[action[x][y]]
        x = x2
        y = y2
        path.append([x,y])
        
    path.reverse()
    
    print('\na) Random Path')
    printpath(path)
    printgrid(policy)
    print('\nIterations : ',current_step,'\n')
            
    return "Random Search Pass" 


# **************************Optimal Planner - A Star *************************
def OptSearch(grid,robot_pose,goal_pose):
   
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[robot_pose[0]][robot_pose[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    
    # defining the cost of motion per cell in the grid
    cost = 1
 
    x = robot_pose[0]
    y = robot_pose[1]
    g = 0 # forward motion cost from source of the robot to current cell position
    h = math.sqrt((x-goal_pose[0])*(x-goal_pose[0])+(y-goal_pose[1])*(y-goal_pose[1])) #heuristic cost from current cell to the goal
    f = g + h #net cost to be minimised
    

    open = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    current_step = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Optimal Search Failed"
        else:
            open.sort()
            open.reverse()
            next = open.pop() #selecting the cell with the least cost function
            
            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
                        
            count += 1
             
            if x == goal_pose[0] and y == goal_pose[1]:
                found = True
            else:
                ##expand the element that comes through and add to the open list
                for i in range(len(move)):
                    x2 = x + move[i][0]
                    y2 = y + move[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = h = math.sqrt((x2-goal_pose[0])*(x2-goal_pose[0])+(y2-goal_pose[1])*(y2-goal_pose[1])) #heuristic cost
                            f2 = g2 + h2
                            open.append([f2,g2,h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i # memorising the action it took to reach there, notice that we dont associate with x and y the from state
                            current_step += 1
                            
                            
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    x = goal_pose[0]
    y = goal_pose[1]
    policy[x][y] = '*'
    
    path = []
    path.append([x,y])
   # we go from goal_pose backwards till we find the robot_poseial position
    while x != robot_pose[0] or y!=robot_pose[1]:
        x2 = x - move[action[x][y]][0]
        y2 = y - move[action[x][y]][1]
        policy[x2][y2] = move_sign[action[x][y]]
        x = x2
        y = y2
        path.append([x,y])
        
    path.reverse()
    
    print('\nb) Optimum Path')
    printpath(path)
    printgrid(policy)
    print('\nIterations : ',current_step,'\n')
           
    return "Optimal Search Pass"



#************************************************** 
    

def printpath(path):
    print(path[0],end=' ')           
    for i in range(1,len(path)):
       print('->',end=' ') 
       print(path[i],end=' ')
    print('.')
    
def printgrid(grid):
    print('\nGrid = ')         
    for i in range(len(grid)):
       print(grid[i])
    
#************************************************
# Main Program       

      
if robot_pose[0] < 0 and robot_pose[0] >= len(world_state) and robot_pose[1] > 0 and robot_pose[1] >= len(world_state[0]):
    print('robot_pose out of world state!')

elif goal_pose[0] < 0 and goal_pose[0] >= len(world_state) and goal_pose[1] > 0 and goal_pose[1] >= len(world_state[0]):
    print('goal_pose out of world state!')
    
else:

    #calling the random function:
    print(RandomSearch(world_state,robot_pose,goal_pose))
        
    #calling the optimum function:
    print(OptSearch(world_state,robot_pose,goal_pose))
    #









