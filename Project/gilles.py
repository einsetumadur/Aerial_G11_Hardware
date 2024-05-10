# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

# Global variables
on_ground = True
height_desired = 0.5
timer = None
startpos = None
timer_done = None
dronewidth = 0.06
DEBUG = False
PRINT = True
WAITIMG = 1
timeoutsec = 15
senseforce = False

# Occupancy map based on distance sensor
min_x, max_x = -2.0, 3.0 # meter
min_y, max_y = -1.5, 1.5 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.05  # meter
conf = 0.2 # certainty given by each measurement
dronekernel = int(dronewidth/res_pos)*2 + 1

# flight phase
mode = 'scan' # 'scan', 'goto', 'land', 'return'
next_waypoint = np.zeros((3,1)) # x,y,z
startpad = np.array([0.0, 0.0, height_desired])
landingpad = np.array([4.0, 1.5, 0.0])
found_landingpad = False
scanrate = 0.9 # rad/s
landvel = 0.3 # m/s

# This is the main function where you will implement your control algorithm
def get_command(sensor_data, dt):
    
    global on_ground, startpos,map,mode,next_waypoint,startpad,height_desired,scanrate
    occupancy_map(sensor_data)

    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]    
        startpad = np.array([startpos[0], startpos[1], height_desired])

    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, height_desired, 0.0]
        return control_command
    else:
        on_ground = False

    # ---- YOUR CODE HERE ----
    if DEBUG:
        showmap(sensor_data)

    sensor_data['yaw'] = clip_angle(sensor_data['yaw'])
            
    # switch case of flight modes
    #################################### SCAN ####################################
    if mode == 'scan':
        control_command = [0,0,height_desired,0.5]
        if(sensor_data['yaw'] > np.pi-0.2):
            mode = 'goto'
            if PRINT:
                print("going to next waypoint")
            pot = potentialfield(map,groundmap,kernsize=dronekernel,exploretype='forward')
            next_waypoint = get_next_waypoint(pot)
        else:
            return control_command

    #################################### GOTO ####################################            
    elif mode == 'goto':
        control_command,arrived = goto_waypoint(sensor_data, next_waypoint)
        if sensor_data['x_global'] > 3.5:
            mode = 'search'
        elif arrived:
            mode = 'scan'

    #################################### SEARCH ####################################
    elif mode == 'search':
        control_command,next_waypoint = detect_landingpad(sensor_data,dt)

    #################################### PROBE ####################################
    elif mode == 'probe':
        control_command = probe_landingpad(sensor_data)
        if control_command is None:
            print("probe failed : {}".format(probe_landingpad.probemode))
            control_command = [0,0,height_desired,0]
            mode = 'land'

    #################################### LAND ####################################
    elif mode == 'land':
        if sensor_data['range_down'] < 0.02:
            if PRINT:
                print("landed")
            else:
                height_desired = 1
            mode = 'return'

        height_desired -= landvel*dt
        control_command = [0,0,height_desired,0]

    #################################### RETURN ####################################
    elif mode == 'return':
        control_command,arrived = goto_waypoint(sensor_data,startpad)
        if arrived:
            mode = 'finish'

    #################################### FINISH ####################################
    elif mode == 'finish':
        height_desired -= landvel*dt
        control_command = [0,0,height_desired,0]
    else :
        control_command = [0,0,0,0]
    
    #print(control_command)
    return control_command # Ordered as array with: [v_forward_cmd, v_left_cmd, alt_cmd, yaw_rate_cmd]

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied
groundmap = np.zeros_like(map)

def occupancy_map(sensor_data):
    global map,groundmap, t
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
            if idx_x < 1 or idx_x >= map.shape[0]-1 or idx_y < 1 or idx_y >= map.shape[1]-1 or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
                #and bordering cells
                map[idx_x-1, idx_y-1] += conf
                map[idx_x-1, idx_y] += conf
                map[idx_x-1, idx_y+1] += conf
                map[idx_x, idx_y-1] += conf
                map[idx_x, idx_y+1] += conf
                map[idx_x+1, idx_y-1] += conf
                map[idx_x+1, idx_y] += conf
                map[idx_x+1, idx_y+1] += conf
                
            else:
                map[idx_x, idx_y] -= conf
                #and bordering cells
                map[idx_x-1, idx_y-1] -= conf
                map[idx_x-1, idx_y] -= conf
                map[idx_x-1, idx_y+1] -= conf
                map[idx_x, idx_y-1] -= conf
                map[idx_x, idx_y+1] -= conf
                map[idx_x+1, idx_y-1] -= conf
                map[idx_x+1, idx_y] -= conf
                map[idx_x+1, idx_y+1] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    gndidx = pos2mapidx([pos_x,pos_y])
    if gndidx[0] >= 0 and gndidx[0] < groundmap.shape[1] and gndidx[1] >= 0 and gndidx[1] < groundmap.shape[0]:
        groundmap[gndidx[1]][gndidx[0]] = sensor_data['range_down']

    return map

def detect_landingpad(sensor_data,dt):
    global mode,groundmap,map,height_desired
    hth = 0.06
    if not hasattr(detect_landingpad, "hasarrived"):
        detect_landingpad.hasarrived = True
        detect_landingpad.expwaypoint = [sensor_data['x_global'],sensor_data['y_global'],height_desired]
        detect_landingpad.timer = 0
        detect_landingpad.lasttime = 0
        detect_landingpad.counter = 0
    control_command = [0,0,height_desired,0]
    if DEBUG:
        cv2.imshow('grndheight',groundmap*122)
        cv2.waitKey(1)

    posidx = pos2mapidx([sensor_data['x_global'],sensor_data['y_global']])
    posidx = [posidx[0],posidx[1]]

    if ((sensor_data['range_down'] < (height_desired-hth)) and sensor_data['x_global'] > 3.5 and (abs(sensor_data['pitch']) < np.deg2rad(10)) and (abs(sensor_data['roll']) < np.deg2rad(10))):
        detect_landingpad.counter += 1
        if detect_landingpad.counter > 40:
            detect_landingpad.counter = 0
            if PRINT:
                    print("landing pad cond {} at {}".format(sensor_data['range_down'],posidx))
            mode = 'probe'
    else:
        detect_landingpad.counter = 0
    
    if detect_landingpad.hasarrived:
        detect_landingpad.lasttime = detect_landingpad.timer
        if PRINT:
                print("computing field")
        pot = potentialfield(map,groundmap,kernsize=dronekernel,safetymult=4,exploretype='ground',pos=posidx,finallandzone=True)
        detect_landingpad.expwaypoint = get_next_waypoint(pot)
        if PRINT:
                print("expolring ground map next waypoint: ",detect_landingpad.expwaypoint)
        detect_landingpad.hasarrived = False
    else:
        if detect_landingpad.timer - detect_landingpad.lasttime > timeoutsec:
            if PRINT:
                print("timeout")
            detect_landingpad.lasttime = detect_landingpad.timer
            pot = potentialfield(map,groundmap,kernsize=dronekernel,safetymult=4,exploretype='ground',pos=posidx,finallandzone=True)
            detect_landingpad.expwaypoint = get_next_waypoint(pot)
            if PRINT:
                print("expolring ground map next waypoint: ",detect_landingpad.expwaypoint)
            detect_landingpad.hasarrived = False
    
    control_command,detect_landingpad.hasarrived = goto_waypoint(sensor_data,detect_landingpad.expwaypoint)

    detect_landingpad.timer += dt
    return control_command,detect_landingpad.expwaypoint

def probe_landingpad(sensor_data):
    global mode,landingpad
    explorerate = 0.1
    if not hasattr(probe_landingpad, "probemode"):
        probe_landingpad.probemode = 'right' # 'right', 'left', 'front', 'back', 'done', 'failed'
        probe_landingpad.returning = True
        probe_landingpad.counter = 0
        pitchoffsetx = np.cos(sensor_data['yaw'])*(-np.sin(sensor_data['roll']) -np.sin(sensor_data['pitch']))*sensor_data['range_down']
        pitchoffsety = np.sin(sensor_data['yaw'])*(-np.sin(sensor_data['roll']) -np.sin(sensor_data['pitch']))*sensor_data['range_down']
        probe_landingpad.entrypoint = [sensor_data['x_global']+pitchoffsetx,sensor_data['y_global']+pitchoffsety,height_desired]
        probe_landingpad.sidept = [sensor_data['y_global'],sensor_data['y_global'],sensor_data['x_global'],sensor_data['x_global']] # righty, lefty, frontx, backx
    global mode,landingpad
    allignyaw = 0.05

    if abs(sensor_data['yaw']) > allignyaw:
        control_command = [0,0,height_desired,-sensor_data['yaw']*1.5]
        return control_command
    elif probe_landingpad.probemode != 'failed' and np.sqrt((sensor_data['x_global'] - probe_landingpad.entrypoint[0])**2 + (sensor_data['y_global'] - probe_landingpad.entrypoint[1])**2) > 0.4:
        probe_landingpad.probemode = 'failed'
        if PRINT:
            print("probe failed")
        return [0,0,height_desired,0]
    elif probe_landingpad.probemode == 'failed':
        control_command,arrived = goto_norotate(sensor_data,probe_landingpad.entrypoint,epsilon=0.05)
        if arrived:
            mode = 'land'
            return [0,0,height_desired,0]
        else:
            return control_command
    else:
        if not probe_landingpad.returning and sensor_data['range_down'] > height_desired + 0.09:
            if probe_landingpad.counter < 5:
                probe_landingpad.counter += 1
            else:
                if probe_landingpad.probemode == 'right' and sensor_data['y_global'] < probe_landingpad.sidept[0]:        
                        probe_landingpad.sidept[0] = sensor_data['y_global']
                        probe_landingpad.returning = True
                        probe_landingpad.probemode = 'left'
                elif probe_landingpad.probemode == 'left' and sensor_data['y_global'] > probe_landingpad.sidept[0]+0.1:
                        probe_landingpad.sidept[1] = sensor_data['y_global']
                        landingpad[1] = (probe_landingpad.sidept[0] + probe_landingpad.sidept[1])/2
                        probe_landingpad.entrypoint[1] = landingpad[1]
                        if abs(probe_landingpad.sidept[0] - probe_landingpad.sidept[1]) > 0.2:
                            probe_landingpad.returning = True
                            probe_landingpad.probemode = 'back'
                elif probe_landingpad.probemode == 'back' and sensor_data['x_global'] < probe_landingpad.sidept[2]:
                        probe_landingpad.sidept[2] = sensor_data['x_global']
                        probe_landingpad.returning = True
                        probe_landingpad.probemode = 'front'
                elif probe_landingpad.probemode == 'front' and sensor_data['x_global'] > probe_landingpad.sidept[2]+0.1:
                        probe_landingpad.sidept[3] = sensor_data['x_global']
                        if abs(probe_landingpad.sidept[0] - probe_landingpad.sidept[1]) > 0.2:
                            # end of sequence
                            landingpad[0] = (probe_landingpad.sidept[2] + probe_landingpad.sidept[3])/2
                            landingpad[2] = height_desired
                            probe_landingpad.probemode = 'done'

        if probe_landingpad.probemode == 'done':
            control_command,arrived = goto_norotate(sensor_data,landingpad,epsilon=0.05)
            if arrived:
                mode = 'land'
                return [0,0,height_desired,0]
            else:
                return control_command
        elif probe_landingpad.returning:
            control_command,arrived = goto_norotate(sensor_data,probe_landingpad.entrypoint,epsilon=0.05)
            if arrived:
                probe_landingpad.returning = False
                return [0,0,height_desired,0]
            else:
                return control_command
        elif probe_landingpad.probemode == 'right':
            control_command = [0,-explorerate,height_desired,0]
            return control_command
        elif probe_landingpad.probemode == 'left':
            control_command = [0,explorerate,height_desired,0]
            return control_command
        elif probe_landingpad.probemode == 'front':
            control_command = [explorerate,0,height_desired,0]
            return  control_command
        elif probe_landingpad.probemode == 'back':
            control_command = [-explorerate,0,height_desired,0]
            return control_command
        else:
            print("WTF")
            control_command = [0,0,height_desired,0]

def goto_norotate(sensor_data, waypoint,epsilon=0.1):
    ex = waypoint[0] - sensor_data['x_global']
    ey = waypoint[1] - sensor_data['y_global']
    ez = waypoint[2] - sensor_data['range_down']

    kx = 0.5
    ky = 0.5
    kz = 0.5
    kyaw = 0.5
    v_forward = ex*kx
    v_side = ey*ky
    alt_desired = waypoint[2]

    command = [v_forward, v_side, alt_desired, -sensor_data['yaw']*kyaw]
    arrived = (np.sqrt(ex**2 + ey**2 + ez**2) < epsilon) and (np.abs(ez) < epsilon)

    return command,arrived

def goto_waypoint(sensor_data, waypoint,epsilon=0.05,mapborder=False):
    global map,mode
    kp_forward = 1.5
    kp_yaw = 0.3
    kp_range = 0.1
    max_speed = 0.33

    if not hasattr(goto_waypoint, "counter"):
        goto_waypoint.counter = 0
        goto_waypoint.lasterror = 0

    # Calculate the desired yaw angle
    dx = waypoint[0] - sensor_data['x_global']
    dy = waypoint[1] - sensor_data['y_global']
    yaw_desired = np.arctan2(dy, dx)*kp_yaw

    # Calculate the desired forward velocity
    if(np.abs(np.arctan2(dy, dx) - sensor_data['yaw']) < np.deg2rad(10)):
        v_forward = np.sqrt(dx**2 + dy**2)*kp_forward
    else:
        v_forward = 0.0

    # Calculate the desired left velocity
    v_side = 0.0

    # Calculate the desired altitude
    alt_desired = waypoint[2]

    # Calculate the desired yaw rate
    yaw_rate = yaw_desired - sensor_data['yaw']
    if yaw_rate < -np.pi*1.1:
        yaw_rate = 2*np.pi - yaw_rate
    elif yaw_rate > np.pi*1.1:
        yaw_rate = -2*np.pi + yaw_rate
    yaw_rate *= kp_yaw

    v_forward = np.clip(v_forward, 0, max_speed)

    # add obstacle avoidance contribution
    if goto_waypoint.counter == 0:
        obsmap = np.copy(map)
        if mapborder:
            obsmap[0, :] = -1
            obsmap[-1, :] = -1
            obsmap[:, 0] = -1
            obsmap[:, -1] = -1

        _,obsmap = cv2.threshold(obsmap,-0.1,1, cv2.THRESH_BINARY_INV)
        dist,angdiff = get_closest_point(obsmap,sensor_data)
        
        danger = np.clip(pow(1/(dist+1),10),0.0,max_speed)
        
        if dist < 0.15: # near obstacle
            v_forward = -np.cos(angdiff)*danger
            v_side = -np.sin(angdiff)*danger
            # if angdiff >= 0:
            #     yaw_rate -= 1
            # else:
            #     yaw_rate += 1

            if PRINT:
                print("danger: {:.2f} at ({:.2f}m {:.2f}rad) vf:{:.3f} vs:{:.3f}".format(danger,dist,angdiff,-np.cos(angdiff)*danger,-np.sin(angdiff)*danger))
            if DEBUG:
                cv2.waitKey(1)
        else:
            v_forward -= np.cos(angdiff)*danger*kp_range
            v_side -= np.sin(angdiff)*danger*kp_range
            v_forward = np.clip(v_forward, -max_speed, max_speed)
            v_side = np.clip(v_side, -max_speed, max_speed)

        goto_waypoint.counter = 0
    else:
        goto_waypoint.counter -= 1

    if abs(sensor_data['pitch']) > 0.06:
        v_forward = 0
    if abs(sensor_data['roll']) > 0.06:
        v_forward = 0

    arrived = (np.sqrt(dx**2 + dy**2) < 0.1 and np.abs(yaw_desired - sensor_data['yaw']) < epsilon)

    return [v_forward, v_side, alt_desired, yaw_rate],arrived

def get_closest_point(obsmap,sensor_data):
    posidx = pos2mapidx([sensor_data['x_global'],sensor_data['y_global']])
    posidx = [posidx[0],posidx[1]]
    mindist = 1000

    obs_indices = np.argwhere(obsmap == 1)
    for obspx in obs_indices:
        #print("obsidx",obspx)
        dist = np.sqrt((obspx[0] - posidx[1])**2 + (obspx[1] - posidx[0])**2)
        if dist < mindist:
            mindist = dist
            closeidx = obspx
    angtoobs = np.arctan2(closeidx[1] - posidx[0],closeidx[0] - posidx[1])
    angdiff = clip_angle(angtoobs - sensor_data['yaw'])

    if True:
        obsmap = cv2.circle(obsmap,posidx, 2, 1, 1)
        obsmap = cv2.line(obsmap,posidx,(closeidx[1],closeidx[0]),1,1)
        cv2.imshow('obsmap',cv2.resize(obsmap,(300,500),interpolation=cv2.INTER_LINEAR))
        cv2.waitKey(WAITIMG)
    # if PRINT:
    #     print("mindist:{:.2f} relang:{:.2f} = (angtoobs:{:.2f} - yaw:{:.2f})".format(mindist*res_pos,np.rad2deg(angdiff),np.rad2deg(angtoobs),np.rad2deg(sensor_data['yaw'])))

    return mindist*res_pos,angdiff 

def clip_angle(angle):
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    if angle < -np.pi:
        angle += 2*np.pi
    return angle

def get_next_waypoint(potentialfield:np.ndarray):
    next_waypoint_idx = maxintrestidx(potentialfield)
    next_waypoint = mapidx2pos(next_waypoint_idx)
    #print(next_waypoint)
    next_waypoint = np.append(next_waypoint,height_desired)
    return next_waypoint

def showmap(sensor_data):
    global map,next_waypoint
    droneidx = pos2mapidx([sensor_data['x_global'], sensor_data['y_global']])
    waypointidx = pos2mapidx(next_waypoint).reshape(-1)
    showmap = cv2.merge([np.zeros_like(map), np.zeros_like(map), map*255])
    showmap = cv2.circle(showmap,droneidx, 5, (0,255,0), 2)
    showmap = cv2.circle(showmap,waypointidx, 5, (255,0,0), -1)
    showmap = cv2.resize(showmap,(300,500),interpolation=cv2.INTER_LINEAR)
    cv2.imshow('map', showmap)
    cv2.waitKey(1)

def mapidx2pos(idx):
    return np.array([idx[0]*res_pos + min_x, idx[1]*res_pos + min_y])

def pos2mapidx(pos):
    return np.array([(pos[1] - min_x)/res_pos, (pos[0] - min_y)/res_pos]).astype(int)

def potentialfield(map:np.ndarray,groundmap,kernsize=5,safetymult=3,exploretype='forward',pos=[0,0],finallandzone=False,startlandzone=False,pad=5,mapborder=True,randnoise= False):
    structelem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (safetymult*kernsize,safetymult*kernsize))
    if mapborder:
        map[0, :] = -1
        map[-1, :] = -1
        map[:, 0] = -1
        map[:, -1] = -1
    _,mapth = cv2.threshold(map,-0.1,1, cv2.THRESH_BINARY)
    obsfield = cv2.morphologyEx(mapth, cv2.MORPH_ERODE, structelem)
    obsfield = cv2.GaussianBlur(obsfield, (kernsize,kernsize), 0)
    sidefield = np.zeros_like(map)
    sidefield[pad:-pad][pad:-pad] = 1
    sidefield = cv2.GaussianBlur(sidefield, (kernsize,kernsize), 0)
    if exploretype == 'forward':
        explorefield = np.ones_like(map) * np.arange(0,1,1/map.shape[0]).reshape((map.shape[0],1))
        potentialfield = obsfield * explorefield * sidefield
    elif exploretype == 'ground':
        blurkern = kernsize*safetymult
        pathkern = int(kernsize*1.5)
        if blurkern%2 == 0:
            blurkern += 1
        _,invexp = cv2.threshold(groundmap,0.3,1,cv2.THRESH_BINARY_INV)
        invexp = cv2.morphologyEx(invexp, cv2.MORPH_ERODE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(pathkern,pathkern)))
        gndexplore = cv2.GaussianBlur(invexp, (blurkern,blurkern), 0)
        workfield = np.zeros_like(map)
        workfield = gausscircle(pos,workfield,blurkern,kernsize*2+1)
        if randnoise:
            workfield = workfield * mapnoise(map.shape)
        potentialfield = obsfield * gndexplore * workfield
        if finallandzone:
            potentialfield[:][0:int(3.5/res_pos)] = 0
        if startlandzone:
            potentialfield[:][int(1.5/res_pos):-1] = 0
    else:
        potentialfield = obsfield
    if True:
        cv2.imshow('field', cv2.resize(potentialfield,(300,500),interpolation=cv2.INTER_LINEAR))
        cv2.waitKey(WAITIMG)
    
    return potentialfield

def maxintrestidx(potentialfield:np.ndarray):
    maxintidx = np.unravel_index(np.argmax(potentialfield, axis=None), potentialfield.shape)
    #print(maxintidx)
    return maxintidx

def gausscircle(pos, field, sigout,sigin):
            y,x = pos
            for i in range(field.shape[0]):
                for j in range(field.shape[1]):
                    distance = np.sqrt((i - x)**2 + (j - y)**2)
                    field[i, j] += np.exp(-distance**2 / (2 * sigout**2))
                    field[i, j] -= np.exp(-distance**2 / (2 * sigin**2))
            return field

def gausspoint(pos, field, sig):
            y,x = pos
            for i in range(field.shape[0]):
                for j in range(field.shape[1]):
                    distance = np.sqrt((i - x)**2 + (j - y)**2)
                    field[i, j] += np.exp(-distance**2 / (2 * sig**2))
            return field

def mapnoise(mapshape):
    noise = np.random.randn(mapshape[0],mapshape[1])
    min_val = -3
    max_val = 3
    noise = (noise - min_val) / (max_val - min_val)
    noise = np.clip(noise, 0, 1)
    return noise

if __name__ == "__main__":
    map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos)))
    dronepos = [2,1]
    d = 0.5
    ang = np.pi/4
    map = cv2.circle(map,(int((dronepos[1]-d*np.sin(ang))/res_pos),int((dronepos[0]*d*np.cos(ang))/res_pos)), 4, 1, 1)
    map[0,:]=1
    map[-1,:]=1
    map[:,0]=1
    map[:,-1]=1
    dist,angtoobs = get_closest_point(map,dronepos,0)
    print(dist,angtoobs)

    cv2.waitKey(0)