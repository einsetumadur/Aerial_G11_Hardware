# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import random

# Global variables
on_ground = True
height_desired = 0.4
timer = None
startpos = None
timer_done = None
timery = 0
land_area_found = False
searching_land = False
gostart = False
landing_region = np.zeros((15,30))
# All possible adjacent positions for A* algorithm
adj_pos = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
cost = 0.5 # cost of knowing, if we know there is no obstacle, cost of g distance is devided by 50%
x_ind_search = 0
y_ind_search = 0

# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "range_down": Downward range finder distance (Used instead of Global Z distance)
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "yaw": Yaw angle (rad)

# This is the main function where you will implement your control algorithm
def get_command(sensor_data, dt):
    global on_ground, startpos, timery, state, current_path, next_point, index, sum_ang, ang_diff, current_goal, goal_pos, searching_land, land_area_found, landing_region, gostart
    global x_ind_search, y_ind_search
    timery += dt

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    # cv2.imshow('Camera Feed', camera_data)
    # cv2.waitKey(1)
    # Take off
    if startpos is None:
        state = "initializing"
        startpos = real_map_trans(sensor_data)  
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, height_desired, 0.0]
        return control_command
    else:
        on_ground = False
    # ---- YOUR CODE HERE ----
    on_ground = False
    #update map
    pos_grid = real_map_trans(sensor_data)
    grid_update = occupancy_map(sensor_data)
    down_meas = sensor_data['range_down']
    if searching_land:
        #update landing grid too
        landing_region = update_land_grid(landing_region, grid_update)

    match state:
        case "initializing":
            # Wait for the drone to stabilize
            control_command = [0.0, 0.0, height_desired, 0.0]
            print('Range down = ', sensor_data['range_down'])
            if abs(height_desired - sensor_data['range_down']) < 0.1:
                print('Stabilized!')
                state = 'initial scan'
                sum_ang = 0.0
                current_goal = [49, 15]
                print('Scanning...')

        case 'initial scan':
            #Do a 360 and scan the environment around
            ang_diff = sensor_data['yaw']%(2*np.pi) - sum_ang
            sum_ang += ang_diff
            if sum_ang < (np.pi/2):
                control_command = [0.0, 0.0, height_desired, 0.8]
            else:
                print('Scanning complete!')
                control_command = [0.0, 0.0, height_desired, 0.0]
                print('Calculating path...')
                # Calculate the path to goal point
                current_path = astar(grid_update, pos_grid, current_goal)
                index = 1
                next_point = current_path[index]
                state = 'moving' 

        case "moving":
            #wait for drone to reach next goal
            distance = np.linalg.norm([sensor_data['x_global'] - next_point[0]*res_pos, sensor_data['y_global'] - next_point[1]*res_pos])
            if distance < 0.05:
                control_command = [0.0, 0.0, height_desired, 0.0]
                if index == (len(current_path) - 1):
                    if searching_land:
                        state = 'search landing'
                    elif land_area_found:
                        print('Landing on pad!')
                        state = 'land'
                        return [0.0, 0.0, height_desired, 0.0]
                    elif gostart:
                        gostart = False
                        state = 'back to start'

                else:
                    state = 'next'
            else:
                control_command = drone_control(sensor_data, next_point, dt)
        
        case "next":
            #check next point of path, if blocked, recalculate path, else go to moving
            control_command = [0.0, 0.0, height_desired, 0.0]
            index += 1
            next_point = current_path[index]
            if next_point[0] > 35 and not(searching_land) and not(land_area_found) and not(gostart):
                gostart = True
                state = 'back to start'
                return [0.0, 0.0, height_desired, 0.0]
            else:
                # Check that path is achievable
                if searching_land:
                    if down_meas < 0.95:
                        print('Landing pad found!')
                        goal_pos = pos_grid
                        searching_land = False
                        land_area_found = True
                        current_path = []
                        for adj in [[1,0], [0, 1], [-1, 0], [0, -1]]:
                            pos_path = [goal_pos[0]+adj[0], goal_pos[1]+adj[1]]
                            if check_in_map(grid_update.shape, pos_path) and grid_update[pos_path[0]][pos_path[1]] > -0.5:
                                current_path.append([goal_pos[0]+adj[0], goal_pos[1]+adj[1]])
                        index = 0
                        next_point = current_path[index]
                        state = 'moving'
                    else: 
                        #update landing grid if in land_region
                        if pos_grid[0] > 36:
                            landing_region[pos_grid[0]-35][pos_grid[1]] = 1
                        # also check that last point is still viable
                        # if not, go back to search lading
                        if current_path[-1] == 1:
                            state = 'search landing'
                            return [0.0, 0.0, 0.0, 0.0]
                elif land_area_found:
                    if sensor_data['z_global'] > 1.08 or down_meas < 0.95:
                        print('Landing on pad!')
                        goal_pos = pos_grid
                        state = 'land'
                        return [0.0, 0.0, height_desired, 0.0]

                for i in range(len(current_path)-index):
                    check_point = current_path[index+i]
                    if grid_update[check_point[0]][check_point[1]] < -0.6:
                        # Recalculate path
                        current_path = astar(grid_update, pos_grid, current_goal)
                        index = 1
                        if current_path == False:
                            state = 'searching land'
                            return [0.0, 0.0, height_desired, 0.0]
                        next_point = current_path[index]
                        state = 'moving' 
                        return [0.0, 0.0, height_desired, 0.0]
            
            state = 'moving'
            return [0.0, 0.0, height_desired, 0.0]


        case "goal reached":
            # if goal is reached, stop
            control_command = [0.0, 0.0, 0.5, 0.0]
        
        case 'search landing':
            # If here, then we reached the wanted point
            if down_meas < 0.95:
                print('Landing pad found!')
                goal_pos = pos_grid
                searching_land = False
                land_area_found = True
                current_path = []
                for adj in [[1,0], [0, 1], [-1, 0], [0, -1]]:
                    pos_path = [goal_pos[0]+adj[0], goal_pos[1]+adj[1]]
                    if check_in_map(grid_update.shape, pos_path) and grid_update[pos_path[0]][pos_path[1]] > -0.5:
                        current_path.append([goal_pos[0]+adj[0], goal_pos[1]+adj[1]])
                index = 0
                state = 'moving'
            else:
                control_command = [0.0, 0.0, height_desired, 0.0]
                #pick random point in the grid landing region and check that is is higher than zero and that it is not one in landing grid
                
                while(landing_region[x_ind_search*3+1][y_ind_search*3+1] or pos_grid==[x_ind_search*3+1+35, y_ind_search*3+1]):
                    if y_ind_search % 2 == 0:  # Even row (move left to right)
                        if x_ind_search < 4:
                            x_ind_search += 1
                        else:
                            y_ind_search += 1
                    else:  # Odd row (move right to left)
                        if x_ind_search > 0:
                            x_ind_search -= 1
                        else:
                            y_ind_search += 1
                        
                    if y_ind_search == 10:
                        state = 'error'
                        return [0.0, 0.0, height_desired, 0.0]
                # calculate path to it
                current_goal = [x_ind_search*3+1+35, y_ind_search*3+1]
                current_path = astar(grid_update, pos_grid, current_goal)
                index = 1
                next_point = current_path[index]
                state = 'moving' 

            return [0.0, 0.0, height_desired, 0.0]

    
        case 'land':
            control_command = [0.0, 0.0, 0.0, 0.0]
            # land and wait 1 sec, then go to back to start pos
            if down_meas < 0.05:
                print('Landed!')
                gostart = True
                land_area_found = False
                state = 'back to start'
                current_goal = startpos
            else:
                return [0.0, 0.0, sensor_data['range_down']-10*dt, 0.0]
        
        case 'back to start':
            if down_meas < height_desired-0.03 and gostart:
                return [0.0, 0.0, height_desired, 0.0]
            elif gostart:
                current_path = astar(grid_update, pos_grid, startpos)
                index = 1
                state = 'moving'
                return [0.0, 0.0, height_desired, 0.0]
            elif down_meas > 0.05:
                return [0.0, 0.0, sensor_data['range_down']-10*dt, 0.0]
            else:
                state = 'stop'
                control_command = [0.0, 0.0, 0.0, 0.0]
        
        case 'error':
            # only entered if it didn't find path or landing pad
            control_command = [0.0, 0.0, 0.0, 0.0]
        
        case 'stop':
            control_command = [0.0, 0.0, 0.0, 0.0]




      
    

    
    return control_command # Ordered as array with: [v_forward_cmd, v_left_cmd, alt_cmd, yaw_rate_cmd]


# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.1 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the current_setpoint is within the map
            if not(check_in_map(map.shape, [idx_x,idx_y])) or dist > range_max:
                break

            # update the map
            if dist < measurement:
                adj_blocked = False
                for p in adj_pos:
                    if check_in_map(map.shape, [idx_x+p[0],idx_y+p[1]]):
                        if map[idx_x+p[0]][idx_y+p[1]] <= -1.0:
                            adj_blocked = True

                if not(adj_blocked):
                    map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                for p in adj_pos:
                    if check_in_map(map.shape, [idx_x+p[0],idx_y+p[1]]):
                        if map[idx_x+p[0]][idx_y+p[1]] >= 0.0 and map[idx_x, idx_y] <= -1.0:
                            map[idx_x+p[0]][idx_y+p[1]] = -0.5
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
        plt.close()
    t +=1

    return map


# Control from the exercises
index_current_setpoint = 0
def path_to_setpoint(path,sensor_data,dt):
    global on_ground, height_desired, index_current_setpoint, timer, timer_done, startpos

    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]    
    if on_ground and sensor_data['range_down'] < 0.49:
        current_setpoint = [startpos[0], startpos[1], height_desired, 0.0]
        return current_setpoint
    else:
        on_ground = False

    # Start timer
    if (index_current_setpoint == 1) & (timer is None):
        timer = 0
        print("Time recording started")
    if timer is not None:
        timer += dt
    # Hover at the final setpoint
    if index_current_setpoint == len(path):
        # Uncomment for KF
        control_command = [startpos[0], startpos[1], startpos[2]-0.05, 0.0]

        if timer_done is None:
            timer_done = True
            print("Path planing took " + str(np.round(timer,1)) + " [s]")
        return control_command

    # Get the goal position and drone position
    current_setpoint = path[index_current_setpoint]
    x_drone, y_drone, z_drone, yaw_drone = sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down'], sensor_data['yaw']
    distance_drone_to_goal = np.linalg.norm([current_setpoint[0] - x_drone, current_setpoint[1] - y_drone, current_setpoint[2] - z_drone, clip_angle(current_setpoint[3]) - clip_angle(yaw_drone)])

    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < 0.1:
        # Select the next setpoint as the goal position
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(path):
            current_setpoint = [0.0, 0.0, height_desired, 0.0]
            return current_setpoint

    return current_setpoint

def clip_angle(angle):
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    if angle < -np.pi:
        angle += 2*np.pi
    return angle

class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    
    def __eq__(self, other):
        return self.position == other.position
    
    def is_smaller(self,other):
        return self.f < other.f

    def return_path(self):
        path = []
        current = self
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1]

def astar(grid, start, goal):
    """ This function calculates the most optimal path using A* algorithm and returns the path in a list"""

    start_pos = start
    goal_pos = goal

    startnode = Node(None, start_pos)

    #list of possible path positions
    openset = []
    # list of innaccessible positions
    closedset = []

    openset.append(startnode)


    while len(openset) > 0:


        current = openset[0]
        current_ind = 0

        # check for the smallest f value of the list
        for index, item in enumerate(openset):
            if item.is_smaller(current):
                current = item
                current_ind = index

        # if current is goal than it has been found
        if current.position == goal_pos and openset[-1].f >= current.f:
            return current.return_path()
    

        openset.pop(current_ind)
        closedset.append(current)



        for pos in adj_pos:

            child = Node(current, [current.position[0]+pos[0], current.position[1]+pos[1]])

            #check that it is in map
            if not(check_in_map(grid.shape, child.position)):
                continue

            #check that terrain is passable
            if grid[child.position[0]][child.position[1]] <= -0.8:
                continue
            
            #continue if child is in closed list
            if child in closedset:
                continue

            #calc scores
            
            child.g = current.g + (1-cost*grid[child.position[0]][child.position[1]])*sum([(child.position[i] - current.position[i])**2 for i in range(2)])**0.5
            child.h = sum([(child.position[i] - current.position[i])**2 for i in range(2)])**0.5
            child.f = child.g + child.h

            #continue if child is already in open list
            #add child to open list if score is less than current
            if child in openset:
                i = openset.index(child)
                if child.g < openset[i].g:
                    openset[i] = child
            else:
                openset.append(child)
    return False
old_error = 0
old_dist = 0
def drone_control(sensor_data, next_point, dt):
    global old_error, old_dist
    real_point = np.array(next_point)*res_pos
    yaw = sensor_data['yaw']
    Kp_yaw = 9
    Kd_yaw = 1.9
    Kp_v = 4
    Kd_v = 0.07

    diff_vect = [real_point[0] - sensor_data['x_global'], real_point[1] - sensor_data['y_global']]
    distance = np.linalg.norm(diff_vect)



    angle_error = (np.arctan2(diff_vect[1], diff_vect[0])-yaw)
    
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi                                                                                                                                                                                               

    
    yaw_rate = Kp_yaw * angle_error + Kd_yaw*(angle_error-old_error)/dt
    v = Kp_v * distance + Kd_v*(distance-old_dist)/dt
    v_x = v*np.cos(angle_error)
    v_y = v*np.sin(angle_error)
    old_dist = distance
    old_error = angle_error
    
    return [0.1, 0.0, height_desired, yaw_rate]

def check_in_map(grid_len, pos):
    if (pos[0] < 0 or pos[0] > (grid_len[0]-1) or pos[1] < 0 or pos[1] > (grid_len[1]-1)):
        return False
    else:
        return True

def real_map_trans(sensor_data):
    return [int(np.round(sensor_data['x_global']/res_pos,0)), int(np.round(sensor_data['y_global']/res_pos,0))]

def update_land_grid(land_grid, grid):
    for i in range(land_grid.shape[0]-1):
        for j in range(land_grid.shape[1]-1):
            if grid[i+35][j] < -0.4 and land_grid[i][j] == 0:
                land_grid[i][j] = 1
    
    return land_grid


            