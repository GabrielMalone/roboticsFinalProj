from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np
# --------------------------------------------------------------
# apple finding robot --- hopefully
# --------------------------------------------------------------
class myRobot():
    # ----------------------------------------------------------
    def __init__(self):
        # connec to CoppeliaSim
        self.client = RemoteAPIClient()
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
        image, resolution = self.sim.getVisionSensorImg(self.camera)
        # convert image  bytes to numpy array
        image = np.frombuffer(image, dtype=np.uint8)
        image = image.reshape(resolution[1], resolution[0], 3)
        # convert RGB to BGR for OpenCV
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # ----------------------------------------------------------






