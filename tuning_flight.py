
from dronekit import *
from camera_control_helpers import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--run_name', type=str, default='')
args = parser.parse_args()

logger = Logger(args.run_name)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=False) # dont wait for ready, bench test
gains = np.array([0.75,0.1,0,0,1.2,0.2,3.7215,1.1307])

camera = Camera(15,logger) # 15 = human 5 = bottle
arm_and_takeoff(vehicle, logger, 2.5)

mode = "HOVER"
logger.write_log("Starting 5s Hover")
logger.write_log("MODE: HOVER")

goal_xyz_D = np.array([[0],[0],[0]])
psi_rel_deg = np.array([0])
goal_xyz_D_last = np.array([[0],[0],[0]])
psi_rel_deg_last = np.array([0])
max_time_between_detections = timedelta(seconds = 1)
flag = False
t_end = time.time() + 5

while time.time() < t_end:
    log_telemetry(vehicle, logger)
    goal_xyz_D = np.array([[0],[0],[-2]])
    goal_xyz_D_last = goal_xyz_D
    psi_rel_deg_last = psi_rel_deg
    if goal_xyz_D is not None: 
        send_control_msgs_position(vehicle, logger, goal_xyz_D, send_command = True)

logger.write_log("Starting 30s tracking")  

t_end = time.time() + 30

logger.write_log("MODE: TRACK")
mode = "TRACK"
switch_flag = 0

while time.time() < t_end:
    log_telemetry(vehicle, logger)
    goal_xyz_D, psi_rel_deg = camera.get_goal_state(logger)
    # mode switch check 
    
    t = datetime.now()
    if camera.target_detected == True:
        if switch_flag == 1:
            logger.write_log("MODE: TRACK")
            switch_flag = 0
        mode = "TRACK"
    elif ((t - camera.time_of_prev_detection) > max_time_between_detections) and (mode == "TRACK"):
        logger.write_log("Detection Timeout, switching modes...")
        mode = "HOVER"
        switch_flag = 1
        logger.write_log("MODE: HOVER")
        # zero body rates
        goal_xyz_D = np.array([[0],[0],[-0.1]])
        psi_rel_deg = np.array([0])
    elif ((t - camera.time_of_prev_detection) < max_time_between_detections) and (mode == "TRACK") and (camera.target_detected == True):
        # less than one second of no detection, need to repeat last command
        print('Buffer last command...')
        goal_xyz_D = goal_xyz_D_last
        psi_rel_deg = psi_rel_deg_last
    else:
        # mode = Hover and waiting for next detection
        goal_xyz_D = np.array([[0],[0],[-0.1]])
        psi_rel_deg = np.array([0])

    goal_xyz_D_last = goal_xyz_D
    psi_rel_deg_last = psi_rel_deg
    if goal_xyz_D is not None: 
        send_control_msgs(vehicle, logger, goal_xyz_D, psi_rel_deg, gains,flag,send_command = True)
        flag = not(flag)

logger.write_log("MODE: HOVER (final)")
t_end = time.time() + 5
while time.time() < t_end:
    log_telemetry(vehicle, logger)
    goal_xyz_D = np.array([[0],[0],[0]])
    psi_rel_deg = np.array([0])
    goal_xyz_D_last = goal_xyz_D
    psi_rel_deg_last = psi_rel_deg
    if goal_xyz_D is not None: 
        send_control_msgs(vehicle, logger, goal_xyz_D, psi_rel_deg, gains, flag,send_command = True)
print("Closing Vehicle")
vehicle.close()
