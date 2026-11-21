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
import math
import robotHelpers
# --------------------------------------------------------------
# apple finding robot --- hopefully
# --------------------------------------------------------------
class myRobot():
    # ----------------------------------------------------------
    def __init__(self):
        # connect to CoppeliaSim
        self.client         = RemoteAPIClient()
        # requeset access to the sim api and its methods 
        self.sim            = self.client.require('sim')
        # get robot components
        self.robot          = self.sim.getObject('/PioneerP3DX')
        self.camera         = self.sim.getObject('/PioneerP3DX/visionSensor')
        # driving stuff
        self.left_motor     = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.right_motor    = self.sim.getObject('/PioneerP3DX/rightMotor')
        self.wheel_radius   = robotHelpers.getWheelRadius(self)
        self.L              = robotHelpers.getWheelLength(self)
        self.heading_gain   = 1.0
        self.angular_gain   = 1.2
        # tree calibration stuff
        self.bestDifference = float("inf")
        self.bestHeading = None
        self.treeScanComplete = False
        self.treeAligned = False
        self.scanStartHeading = None
        self.previousScanHeading = None
        self.scanRotation = 0.0
        self.lastTreeOmega = -0.12
        # apple stuff
        self.appleRadius = robotHelpers.averageAppleRadius(self)
        self.appleDiameter = self.appleRadius * 2
        # can track apple coordinates here
        self.applePositions = []
        self.tempAppleReadings = []
        # camera stuff
        self.fov = self.sim.getObjectFloatParam(
            self.camera,
            self.sim.visionfloatparam_perspective_angle
            )
        # ratio of sideways distance to forward distance
        # at the edge of the camera's view
        self.tanHalfFov = math.tan(self.fov/2)
        # 1024 set in copelliasim
        self.imageWidth = 1024

        # two imaginary triangles, representing the same camera FOV.
        #1 - World-space triangle: Extends 1 unit forward from the camera. 
        # At that distance, the FOV is 1.155 units wide.
        #2 - Image-plane triangle: Extends 887 pixel-units 
        # forward from the camera. Its width is 1024 pixels.
        # and it's just 1024 a simple question of 1024 = 1.15x

        self.focalLength = (
            self.imageWidth / (2 * math.tan(self.fov / 2))
        )
        # other flags
        self.april_tag_reached = False
        self.moving_around_tree = False
        self.target_distance_to_tree = None
        self.cameraTilted = False
        # actually the x and y top corner 
        # length of april tag in pixels
        self.min_distance_to_april_tag = 250

        # ulstrasonic sensors
        self.usensors = []
        for i in range(16):
            sensor = self.sim.getObject(
                f"/PioneerP3DX/ultrasonicSensor[{i}]"
            )
            self.usensors.append(sensor)
    
    # ---------------------------------------------------------- 
    def start(self):
        self.sim.startSimulation()
    def stop(self):
          self.sim.stopSimulation()
    # ----------------------------------------------------------
    def runSelf(self, ATD):
        # run camera continuously
        while True:
            # --------------------------------------------------
            # image from CoppeliaSim
            # --------------------------------------------------
            image = robotHelpers.getCameraImage(self)
            # any april tags in view?
            corners, ids = ATD.detect(image)
            # for visualization in the sim
            image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
            # --------------------------------------------------
            # if april tag detected move towards it
            # --------------------------------------------------
            if (corners and not self.april_tag_reached):
                robotHelpers.moveTowardsAprilTag(self, corners, image)
            # --------------------------------------------------
            # otherwise spin in a circle
            # and look for tag
            # --------------------------------------------------
            elif not self.april_tag_reached:
                print("just going in circles!")
                robotHelpers.moveSelf(self, v=0, heading_error=0.2)
            # --------------------------------------------------
            # tree / robot alignment 
            # --------------------------------------------------
            if self.april_tag_reached and not self.treeAligned:
                robotHelpers.calibrateRobotTangentToTree(self)
            # --------------------------------------------------
            # once aligned, we can move around the tree
            # --------------------------------------------------
            if self.treeAligned:
                robotHelpers.moveAroundTree(self)
                if not self.cameraTilted:
                    robotHelpers.tiltCameraX(self, 0)
                    self.cameraTilted = True
                # --------------------------------------------------
                # now detect apple and estimate distance 
                # --------------------------------------------------
                detection = robotHelpers.detectMarker(self, image)
                if detection is not None:

                    # kinect depth info
                    boxCenter, applePixelWidth, (centerX, centerY) = detection
                    # all the depth info the camera sees
                    depthImage = robotHelpers.getCameraDepth(self)
                    # depth at center of detected apple
                    appleDepth = depthImage[centerY, centerX]
                    # adjust for radius of typical apple
                    appleDepth += self.appleRadius
                    # apple out of view, not really detecting apple 
                    if appleDepth > 2.0:
                        return None
                    worldDirection, localDirection = robotHelpers.find3dVector(self, boxCenter)
                    # at this point have 3d position of apple relative to camera in meters 
                    # just need to do a transformation from camera to robot then robot to world
                    appleCamerPosition = robotHelpers.Position3DappleRelToCamera(appleDepth, localDirection)
                    appleRobotPosition = robotHelpers.TransformApplePosFromCameraToRobot(self, appleCamerPosition)
                    appleWorldPosition = robotHelpers.TransformAppleFromRobotToWorldPos(self, appleRobotPosition)
                    #robotHelpers.compareEstimateToActual(self, appleWorldPosition)
                    robotHelpers.mapApple(self, appleWorldPosition)

                    print(self.applePositions)
        
            cv2.imshow("robot camera", cv2.flip(image, 1))
            # --------------------------------------------------
            # keyborad input stuff
            # --------------------------------------------------
            # give 30ms for a keyboard check
            # and check for 'q' to quit
            # with bitwise operation
            
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break