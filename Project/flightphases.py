# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

# Constants
desired_height = 0.5
landingpad = [0,0,0]
mode = 'probe'
landheight = desired_height

g_on_ground = True
g_start_pos = None

def get_command(sensor_data, dt):
    camera_data = None
    global g_on_ground, g_start_pos,mode,landheight,landingpad

    # Take off
    if g_start_pos is None:
        g_start_pos = [
            sensor_data["x_global"],
            sensor_data["y_global"],
            sensor_data["range_down"],
        ]
    if g_on_ground and sensor_data["range_down"] < 0.49:
        control_command = [0.0, 0.0, desired_height, 0.0]
        return control_command
    else:
        g_on_ground = False

    #################### START OF FLIGHT ####################
    if mode == 'probe':
        control_command = probe_landingpad(sensor_data)
        return control_command
    elif mode == 'land':
        if sensor_data['range_down'] < 0.02:
            mode = 'return'
        landheight -= 1*dt
        control_command = [0,0,landheight,0]
        return control_command
    elif mode == 'return':
        return [0,0,0,0]
    else:
        return [0,0,desired_height,0]

def probe_landingpad(sensor_data):
    global mode,landingpad
    explorerate = 0.1
    if not hasattr(probe_landingpad, "probemode"):
        probe_landingpad.probemode = 'right' # 'right', 'left', 'front', 'back', 'done', 'failed'
        probe_landingpad.returning = True
        probe_landingpad.counter = 0
        pitchoffsetx = np.cos(sensor_data['yaw'])*(-np.sin(sensor_data['roll']) -np.sin(sensor_data['pitch']))*sensor_data['range_down']
        pitchoffsety = np.sin(sensor_data['yaw'])*(-np.sin(sensor_data['roll']) -np.sin(sensor_data['pitch']))*sensor_data['range_down']
        probe_landingpad.entrypoint = [sensor_data['x_global']+pitchoffsetx,sensor_data['y_global']+pitchoffsety,desired_height]
        probe_landingpad.sidept = [sensor_data['y_global'],sensor_data['y_global'],sensor_data['x_global'],sensor_data['x_global']] # righty, lefty, frontx, backx
    global mode,landingpad
    allignyaw = 0.05

    if abs(sensor_data['yaw']) > allignyaw:
        control_command = [0,0,desired_height,-sensor_data['yaw']*1.5]
        return control_command
    elif probe_landingpad.probemode != 'failed' and np.sqrt((sensor_data['x_global'] - probe_landingpad.entrypoint[0])**2 + (sensor_data['y_global'] - probe_landingpad.entrypoint[1])**2) > 0.4:
        probe_landingpad.probemode = 'failed'
        return [0,0,desired_height,0]
    elif probe_landingpad.probemode == 'failed':
        control_command,arrived = goto_norotate(sensor_data,probe_landingpad.entrypoint,epsilon=0.05)
        if arrived:
            mode = 'land'
            return [0,0,desired_height,0]
        else:
            return control_command
    else:
        if not probe_landingpad.returning and sensor_data['range_down'] > desired_height + 0.09:
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
                            landingpad[2] = desired_height
                            probe_landingpad.probemode = 'done'

        if probe_landingpad.probemode == 'done':
            control_command,arrived = goto_norotate(sensor_data,landingpad,epsilon=0.05)
            if arrived:
                mode = 'land'
                return [0,0,desired_height,0]
            else:
                return control_command
        elif probe_landingpad.returning:
            control_command,arrived = goto_norotate(sensor_data,probe_landingpad.entrypoint,epsilon=0.05)
            if arrived:
                probe_landingpad.returning = False
                return [0,0,desired_height,0]
            else:
                return control_command
        elif probe_landingpad.probemode == 'right':
            control_command = [0,-explorerate,desired_height,0]
            return control_command
        elif probe_landingpad.probemode == 'left':
            control_command = [0,explorerate,desired_height,0]
            return control_command
        elif probe_landingpad.probemode == 'front':
            control_command = [explorerate,0,desired_height,0]
            return  control_command
        elif probe_landingpad.probemode == 'back':
            control_command = [-explorerate,0,desired_height,0]
            return control_command
        else:
            print("WTF")
            control_command = [0,0,desired_height,0]

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
