
import logging
import time
from threading import Timer
import cflib.crtp  # noqa
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import cv2
import numpy as np
from Settings import *
from Logging import Logger

def range2cont(range):
    if range < 0.1:
        if range < 0.1:
            return 1
        else:
            return -((range-0.5)/0.5)
    else:
        return 0

############################## RESET ################################
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers()

last_time = time.time()
get_command_time = time.time()

log = Logger(uri,updaterate=up_period)
cf = log._cf

cf.param.set_value('kalman.resetEstimation', '1')
time.sleep(0.1)
cf.param.set_value('kalman.resetEstimation', '0')
time.sleep(2)

land = False

################################ MAIN ################################


scf = SyncCrazyflie(uri, cf=cf)

starttime = time.time()
while log.is_connected:
    time.sleep(0.01)
    print("[{:.3f}]".format(time.time()-starttime),end=" -> ")
    if log.data_ready():
        sensor_data = log.get_sensor_data()
        sensor_data["x_global"] += 0.2
        sensor_data["y_global"] += 1.0
        if sensor_data["range_up"] < 0.2:
            land = True
        get_command_time = time.time()
        control_command = command_function(sensor_data,np.zeros((10,10)),time.time()-last_time)
        control_command[3] = -control_command[3]
        last_time = time.time()
        print("get_command_time: {:.3f}".format(last_time-get_command_time),end="\t")

    try:
        control_command
    except NameError:
        print("control_command NameError")
        control_command = None
    try:
        sensor_data
    except NameError:
        print("sensor_data NameError")
        sensor_data = None
    
    if time.time() - starttime > MAXTIME or land:
        print("manual timeout",end='\t')
        print(time.time() - starttime)
        if (sensor_data is None):
            break
        cf.commander.send_hover_setpoint(0, 0, 0, sensor_data["range_down"] - LANDRATE)
        if sensor_data["range_down"] < 0.1:
            break

    elif (control_command is not None) and (sensor_data is not None):
        if OBSAVOID:
            front_cont = range2cont(sensor_data["range_front"])
            back_cont = range2cont(sensor_data["range_back"])
            left_cont = range2cont(sensor_data["range_left"])
            right_cont = range2cont(sensor_data["range_right"])
            control_command[0] += -front_cont + back_cont
            control_command[1] += -left_cont + right_cont

        vx = np.clip(control_command[0], -0.3, 0.3)
        vy = np.clip(control_command[1], -0.3, 0.3)
        height = np.clip(control_command[2], 0.1, 2.0)
        yawrate = np.rad2deg(control_command[3])
        print("state: rdown:{:.2f}".format(sensor_data["range_down"]),end="\t")
        print("[vx:{:.2f} vy:{:.2f} yaw:{:.2f} dh:{:.2f}]".format(vx,vy,yawrate,height))
        cf.commander.send_hover_setpoint(vx,vy,yawrate,height)
    else:
        cf.commander.send_hover_setpoint(0, 0, 0, 0.5)


cf.commander.send_stop_setpoint() # Stop the Crazyflie
log._cf.close_link() # Close the link