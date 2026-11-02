# lets us talk to Coppelia Sim 
# https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# python interface to OpenCV (Open Source Computer Vision Library)
# https://docs.opencv.org/5.0/index.html
import cv2
# will use this to detect AprilTags
# apples
# track bojects
# estimate object positions
import numpy as np
# --------------------------------------------------------------
# apple finding robot --- hopefully
# --------------------------------------------------------------
class myRobot():
    # ----------------------------------------------------------
    def __init__(self):
        # connect to CoppeliaSim
        self.client = RemoteAPIClient()
        # requeset access to the sim api and its methods (startSim, stopSim, etc)
        self.sim = self.client.require('sim')
        # get robot components
        self.robot = self.sim.getObject('/PioneerP3DX')
        self.camera = self.sim.getObject('/PioneerP3DX/visionSensor')
    # ---------------------------------------------------------- 
    def start(self):
        self.sim.startSimulation()
    # ----------------------------------------------------------
    def stop(self):
        self.sim.stopSimulation()
    # ----------------------------------------------------------
    def getCameraImage(self):
        # get image from coppeliasim
        # resolution [256, 256]
        image, resolution = self.sim.getVisionSensorImg(self.camera)
        # coppeliasim sends the image as continuous sequence of bytes
        # 255, 0, 124, 0, 255, etc...
        # to turn this into an array of 8 byte units (0,255)
        # [255, 0, 124, 0, 255, etc...]
        image = np.frombuffer(image, dtype=np.uint8)
        # reshape into a 3d array: 256x256 rows cols 
        # containing a 3 item array [r,g, b]
        image = image.reshape(resolution[1], resolution[0], 3)
        # convert RGB to BGR for OpenCV (otherwise colors get swapped)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # ----------------------------------------------------------






