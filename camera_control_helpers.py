from pathlib import Path
# import cv2
import depthai as dai
import numpy as np
import time
# import time
# import argparse
from pymavlink import mavutil
from dronekit import *
import os
from datetime import datetime, timedelta



class Camera:
    def __init__(self, target, logger):
        logger.write_log('Setting Up Camera')
        labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        nnPath = str((Path(__file__).parent / Path('./depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
        fullFrameTracking = False
        # Create pipeline
        pipeline = dai.Pipeline()
        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        objectTracker = pipeline.create(dai.node.ObjectTracker)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        trackerOut = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("preview")
        trackerOut.setStreamName("tracklets")
        # Properties
        camRgb.setPreviewSize(300, 300)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")
        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        spatialDetectionNetwork.setBlobPath(nnPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(10000) # 10 M max detection range
        objectTracker.setDetectionLabelsToTrack([target])  # track target, bottle = 5 person = 15
        objectTracker.setMaxObjectsToTrack(1)  # only track one object
        # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_IMAGELESS)
        # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        camRgb.preview.link(spatialDetectionNetwork.input)
        objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
        objectTracker.out.link(trackerOut.input)
        spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
        spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        spatialDetectionNetwork.out.link(objectTracker.inputDetections)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        self.device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)
        print('USB speed:',self.device.getUsbSpeed())
        self.preview = self.device.getOutputQueue("preview", 4, False)
        self.tracklets = self.device.getOutputQueue("tracklets", maxSize=4, blocking=False)
        self.target_detected = False
        self.time_of_prev_detection = datetime(2024, 1, 1, 1, 1, 1)
        logger.write_log('Camera Setup Complete')


    def get_goal_state(self, logger):
        Rcd = np.array([[0,0,1],[1,0,0],[0,1,0]])
        goal_xyz_D = np.array([[0],[0],[0]])
        psi_rel_deg = np.array([0])
        if self.tracklets.has():
            track = self.tracklets.get()
            trackletsData = track.tracklets
            if trackletsData:
                t = trackletsData[0]
                self.target_detected = True
                self.time_of_prev_detection = datetime.now()
                tgt_xyz_C = np.array([[int(t.spatialCoordinates.x)],[int(t.spatialCoordinates.y)],[int(t.spatialCoordinates.z)]]) * 1/1000
                tgt_xyz_D = Rcd @ tgt_xyz_C # position in meters
                psi_rel_deg = (np.pi/2 - np.arctan2(tgt_xyz_D[0], tgt_xyz_D[1])) * 180 / np.pi
                goal_xyz_D[0] = tgt_xyz_D[0] - 2
                goal_xyz_D[2] = tgt_xyz_D[2]- 0.3
                logger.write_target(f"Target_xyz_D, {str(tgt_xyz_D[0])}, {str(tgt_xyz_D[1])}, {str(tgt_xyz_D[2])}")

        return(goal_xyz_D, psi_rel_deg)

def R_body_to_NED(att_ypr):
    psi = att_ypr[1]
    theta = att_ypr[0]
    phi = att_ypr[2]

    cth = np.cos(theta)
    sth = np.sin(theta)

    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    cphi = np.cos(phi)
    sphi = np.sin(phi)

    R = np.array([[cth*cpsi, -cphi*spsi + sphi*sth*cpsi, sphi*spsi + cphi*sth*cpsi],[cth*spsi, cphi*cpsi + sphi*sth*spsi, -sphi*cpsi + cphi*sth*spsi],[-sth, sphi*cth, cphi*cth]]).reshape(3,3)
    
    return(R)


class Logger:
    def __init__(self,dir_name = ''):
        self.start_time = datetime.now()
        start_time_str = datetime.now().strftime("%d_%H-%M-%S")
        if dir_name == '': 
            run_dir_str = "/home/pi/logs/" + self.start_time + "/"
        else:
            run_dir_str = "/home/pi/logs/" + dir_name + "/"

        os.makedirs(run_dir_str, exist_ok=True)  # Create logs directory if it doesn't exist
        self.location_file = f"{run_dir_str}/loc_{start_time_str}.txt"
        self.velocity_file = f"{run_dir_str}/vel_{start_time_str}.txt"
        self.attitude_file = f"{run_dir_str}/att_{start_time_str}.txt"
        self.control_file = f"{run_dir_str}/con_{start_time_str}.txt"
        self.mode_file = f"{run_dir_str}/mode_{start_time_str}.txt"
        self.log_file = f"{run_dir_str}/log_{start_time_str}.txt"
        self.target_file = f"{run_dir_str}/tgt_{start_time_str}.txt"

    def write_location(self, message):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        with open(self.location_file, "a") as f:
            f.write(f"{dt}, {message}\n")

    def write_target(self, message):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        with open(self.target_file, "a") as f:
            f.write(f"{dt}, {message}\n")

    def write_velocity(self, message):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        with open(self.velocity_file, "a") as f:
            f.write(f"{dt}, {message}\n")
    
    def write_attitude(self, message):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        with open(self.attitude_file, "a") as f:
            f.write(f"{dt}, {message}\n")

    def write_mode(self, message):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        with open(self.mode_file, "a") as f:
            f.write(f"{dt}, {message}\n")

    def write_control(self, goal_xyz_D, psi_rel_deg, vel_input, psidot_des):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        msg = f"Control, {goal_xyz_D[0]}, {goal_xyz_D[1]}, {goal_xyz_D[2]}, {psi_rel_deg}, {vel_input[0]}, {vel_input[1]}, {vel_input[2]}, {psidot_des}"
        with open(self.control_file, "a") as f:
            f.write(f"{dt}, {msg}\n")

    def write_log(self, message):
        current_time = datetime.now()
        dt = (current_time - self.start_time)
        print(message)
        with open(self.log_file, "a") as f:
            f.write(f"{dt}, {message}\n")


# Drone Controlling Functions
def arm_and_takeoff(vehicle, logger, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    logger.write_log("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logger.write_log(" Waiting for vehicle to initialise...")
        time.sleep(1)

    logger.write_log("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        logger.write_log(" Waiting for arming...")
        time.sleep(1)

    logger.write_log("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        logger.write_log("Altitude: " + str(vehicle.location.global_relative_frame.alt))
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            logger.write_log("Reached target altitude")
            break
        time.sleep(1)

def arm_and_sit(vehicle, logger):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    logger.write_log("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logger.write_log(" Waiting for vehicle to be armable...")
        time.sleep(1)

    logger.write_log("Vehicle is Armable, collecting data...")




def send_control_msgs(vehicle, logger, goal_xyz_D, psi_rel_deg, gains,flag, send_command = True):
    # K1 = velocity
    # K2 = position
    # gains = [kx kxd ky kyd kz kzd kp1 kp2]
    
    # xdot_des = gains.k1x*veh_velocity[0] + gains.k2x * tgt_xyz_D[0]
    kx = gains[0]
    kxd = gains[1]
    ky = gains[2]
    kyd = gains[3]
    kz = gains[4]
    kzd = gains[5]
    kp1 = gains[6]
    kp2 = gains[7]
    veh_velocity = np.array(vehicle.velocity)
    veh_location = np.array(vehicle.location.local_frame)
    veh_att = vehicle.attitude
    veh_attitude = np.array([[veh_att.pitch],[veh_att.yaw],[veh_att.roll]])
    R_D_to_NED = R_body_to_NED(veh_attitude)
    goal_xyz_NED = R_D_to_NED @ goal_xyz_D

    xdot_des = kx*goal_xyz_NED[0] - kxd*veh_velocity[0]
    ydot_des = ky*goal_xyz_NED[1] - kyd*veh_velocity[1] 
    zdot_des = kz*goal_xyz_NED[2] - kzd*veh_velocity[2]
    

    # if np.abs(psi_rel_deg) <= 5:
    #     psidot_des = 5
    # elif np.abs(psi_rel_deg) <= 15:
    #     psidot_des = 40
    # else:
    #     psidot_des = 120
    if psi_rel_deg < 0:
        yaw_dir = -1
    else:
        yaw_dir = 0


    psidot_des =  kp1 * kp2 ** np.abs(psi_rel_deg)
    psidot_des = 0
    psi_rel_deg = 2 * np.abs(psi_rel_deg)

    # Limits 2 m/s velocity 120 degree yaw rate
    vel_lim = 2 # m/s
    yaw_rate_lim = 120 # deg/s

    if np.abs(xdot_des) > vel_lim:
        xdot_des = np.sign(xdot_des) * vel_lim
    
    if np.abs(ydot_des) > vel_lim:
        ydot_des = np.sign(ydot_des) * vel_lim

    if np.abs(zdot_des) > vel_lim:
        zdot_des = np.sign(ydot_des) * vel_lim

    if np.abs(psidot_des) > yaw_rate_lim:
        psidot_des = yaw_rate_lim

    vel_input = [xdot_des,ydot_des,zdot_des]



    xyz_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        xdot_des, ydot_des, zdot_des, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    
    yaw_msg = vehicle.message_factory.command_long_encode(
        0, 0,       
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,          
        psi_rel_deg,    
        psidot_des,      #speed deg/s
        yaw_dir,  
        1,          #relative offset 1
        0, 0, 0)

    if send_command:
        if flag:
            vehicle.send_mavlink(xyz_msg)
            vehicle.send_mavlink(xyz_msg)
            time.sleep(1)
        else:
            vehicle.send_mavlink(yaw_msg)
            vehicle.send_mavlink(yaw_msg)
            time.sleep(0.5)
    # send command messages twice to try to ensure delivery
    # logger.write_velocity(f"Velocity, {str(veh_velocity[0])}, {str(veh_velocity[1])}, {str(veh_velocity[2])}")
    # logger.write_location(f"Position, {str(veh_location[0])}, {str(veh_location[1])}, {str(veh_location[2])}")
    logger.write_control(goal_xyz_NED.tolist(), psi_rel_deg, vel_input, psidot_des)


def arm_and_takeoff(vehicle, logger, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    logger.write_log("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logger.write_log(" Waiting for vehicle to initialise...")
        time.sleep(1)

    logger.write_log("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        logger.write_log(" Waiting for arming...")
        time.sleep(1)

    logger.write_log("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        logger.write_log("Altitude: " + str(vehicle.location.global_relative_frame.alt))
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            logger.write_log("Reached target altitude")
            break
        time.sleep(1)

def arm_and_sit(vehicle, logger):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    logger.write_log("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logger.write_log(" Waiting for vehicle to be armable...")
        time.sleep(1)

    logger.write_log("Vehicle is Armable, collecting data...")




def send_control_msgs_position(vehicle, logger, goal_xyz_D, send_command = True):
    # Simple goto command for position, sends command twice in 0.1s
    veh_att = vehicle.attitude
    veh_attitude = np.array([[veh_att.pitch],[veh_att.yaw],[veh_att.roll]])
    R_D_to_NED = R_body_to_NED(veh_attitude)
    goal_xyz_NED = R_D_to_NED @ goal_xyz_D

    xyz_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        goal_xyz_NED[0], goal_xyz_NED[1], goal_xyz_NED[2], # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    

    if send_command:
        vehicle.send_mavlink(xyz_msg)
        time.sleep(0.1)
        vehicle.send_mavlink(xyz_msg)
    # send command messages twice to try to ensure delivery
    # logger.write_velocity(f"Velocity, {str(veh_velocity[0])}, {str(veh_velocity[1])}, {str(veh_velocity[2])}")
    # logger.write_location(f"Position, {str(veh_location[0])}, {str(veh_location[1])}, {str(veh_location[2])}")
    # logger.write_control(goal_xyz_NED.tolist(), psi_rel_deg, vel_input, psidot_des)


def log_telemetry(vehicle, logger):
    veh_attitude = np.array(vehicle.attitude)
    veh_velocity = np.array(vehicle.velocity)
    logger.write_attitude(f"Attitude, {str(veh_attitude)}")
    logger.write_velocity(f"Velocity, {str(veh_velocity[0])}, {str(veh_velocity[1])}, {str(veh_velocity[2])}")
    if vehicle.location.local_frame is not None:
        veh_location = np.array(vehicle.location.local_frame)
        logger.write_location(f"Position, {str(veh_location)}")
     
    
    