
from dronekit import *
from camera_control_helpers import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--run_name', type=str, default='')
args = parser.parse_args()

logger = Logger(args.run_name)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True) # dont wait for ready, bench test
gains = np.array([1,0.1,0,0,0,0,0,0])

camera = Camera(15,logger) # 15 = human 5 = bottle


arm_and_takeoff(vehicle, logger, 2)


# set initial commands to zero rates
# tgt_xyz_D_last = np.array([[0],[0],[0]])
# psi_rel_deg_last = np.array([0])

mode = "HOVER"
logger.write_log("Starting Hover")
logger.write_log("MODE: HOVER")


tgt_xyz_D = np.array([[0],[0],[0]])
psi_rel_deg = np.array([0])
tgt_xyz_D_last = np.array([[0],[0],[0]])
psi_rel_deg_last = np.array([0])
t_end = time.time() + 30

while time.time() < t_end:
    log_telemetry(vehicle, logger)
    tgt_xyz_D_last = tgt_xyz_D
    psi_rel_deg_last = psi_rel_deg
    if tgt_xyz_D is not None: 
        send_control_msgs(vehicle, logger, tgt_xyz_D, psi_rel_deg, gains)

logger.write_log("Starting forward tracking")  
t_end = time.time() + 15
while time.time() < t_end:
    log_telemetry(vehicle, logger)
    tgt_xyz_D, psi_rel_deg = camera.get_goal_state(logger)
    # mode switch check 

    if camera.target_detected == True:
        logger.write_log("MODE: TRACK")
        mode = "TRACK"
    elif ((datetime.now() - camera.time_of_prev_detection) > max_time_between_detections) and (mode == "TRACK"):
        logger.log("Detection Timeout, switching modes...")
        mode = "HOVER"
        logger.write_log("MODE: HOVER")
        # zero body rates
        tgt_xyz_D = np.array([[0],[0],[0]])
        psi_rel_deg = np.array([0])
    elif ((datetime.now() - camera.time_of_prev_detection) < max_time_between_detections) and (mode == "TRACK") and (camera.target_detected == False):
        # less than one second of no detection, need to repeat last command
        tgt_xyz_D = tgt_xyz_D_last
        psi_rel_deg = psi_rel_deg_last

    tgt_xyz_D_last = tgt_xyz_D
    psi_rel_deg_last = psi_rel_deg
    if tgt_xyz_D is not None: 
        send_control_msgs(vehicle, logger, tgt_xyz_D, psi_rel_deg, gains)

logger.write_log("MODE: HOVER (final)")
t_end = time.time() + 30
while time.time() < t_end:
    log_telemetry(vehicle, logger)
    tgt_xyz_D = np.array([[0],[0],[0]])
    psi_rel_deg = np.array([0])
    tgt_xyz_D_last = tgt_xyz_D
    psi_rel_deg_last = psi_rel_deg
    if tgt_xyz_D is not None: 
        send_control_msgs(vehicle, logger, tgt_xyz_D, psi_rel_deg, gains)
print("Closing Vehicle")
vehicle.close()
