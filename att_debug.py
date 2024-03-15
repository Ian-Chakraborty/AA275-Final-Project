
from dronekit import *
from camera_control_helpers import *
# import argparse

# parser = argparse.ArgumentParser()
# parser.add_argument('--run_name', type=str, default='')
# args = parser.parse_args()

# logger = Logger(args.run_name)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=False) # dont wait for ready, bench test



print('Recording Data')
t_end = time.time() + 10
while time.time() < t_end:
    veh_att = vehicle.attitude
    veh_attitude = np.array([[veh_att.pitch],[veh_att.yaw],[veh_att.roll]])
    print(veh_attitude)
    time.sleep(1)

print(type(veh_attitude))
print('Ending Test')


vehicle.close()
