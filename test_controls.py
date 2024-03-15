
from dronekit import *
from camera_control_helpers import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--run_name', type=str, default='')
args = parser.parse_args()

logger = Logger(args.run_name)
vehicle = connect('/dev/serial0', baud=57600, wait_ready=False) # dont wait for ready, bench test
gains = np.array([1,.1,1,.1,1,.1,3.7215,1.1307])


print('Setting up camera')
logger.write_log('Setting Up Camera')
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
        "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
nnPath = '/home/pi/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_5shave.blob'
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
spatialDetectionNetwork.setDepthUpperThreshold(5000)

objectTracker.setDetectionLabelsToTrack([5])  # track only person
objectTracker.setMaxObjectsToTrack(1)  # only track one person
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
# device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

logger.write_log('Camera Setup Complete')



print('Starting 30s control test')
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
    preview = device.getOutputQueue("preview", 4, False)
    tracklets = device.getOutputQueue("tracklets", maxSize=4, blocking=False)
    for i in range(30):
        time.sleep(1)

        Rcd = np.array([[0,0,1],[1,0,0],[0,1,0]])
        # tracklets = self.device.getOutputQueue("tracklets", maxSize=1, blocking=False)
        tgt_xyz_D = []
        psi_rel_deg= []
        if tracklets.has():
            # logger.write_log("Image Detected")
            track = tracklets.get()
            trackletsData = track.tracklets
            if trackletsData:
                t = trackletsData[0]
                print("mark")
                tgt_xyz_C = np.array([[int(t.spatialCoordinates.x)],[int(t.spatialCoordinates.y)],[int(t.spatialCoordinates.z)]]) * 1/1000
                tgt_xyz_D = Rcd @ tgt_xyz_C # position in meters
                tgt_xyz_D[0] = tgt_xyz_D[0] - 2
                tgt_xyz_D[2] = tgt_xyz_D[2] - 1
                psi_rel_deg = (np.pi/2 - np.arctan2(tgt_xyz_D[1], tgt_xyz_D[0])) * 180 / np.pi

        if not isinstance(tgt_xyz_D, list): 
            print(tgt_xyz_D)
            get_control_msgs(vehicle, logger, tgt_xyz_D, psi_rel_deg, gains)
        
        print(i+1)

print("Closing Vehicle")
vehicle.close()
