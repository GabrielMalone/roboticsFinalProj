from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np
import math
import robotHelpers
import time


class myRobot():

    def __init__(self):

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        self.robot = self.sim.getObject('/PioneerP3DX')
        self.camera = self.sim.getObject('/PioneerP3DX/visionSensor')

        # Driving
        self.left_motor = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.right_motor = self.sim.getObject('/PioneerP3DX/rightMotor')
        self.wheel_radius = robotHelpers.getWheelRadius(self)
        self.L = robotHelpers.getWheelLength(self)
        self.heading_gain = 1.0
        self.angular_gain = 1.2

        # Multi-viewpoint scanning
        self.scanState = "searching"
        self.scanStateUntil = 0
        self.scanViewpoint = 0
        self.scanCycle = 0
        self.maxScanViewpoints = 5
        self.scanDuration = 1.0
        self.repositionDuration = 0.5

        # Tree calibration
        self.bestDifference = float("inf")
        self.bestHeading = None
        self.treeScanComplete = False
        self.treeAligned = False
        self.scanStartHeading = None
        self.previousScanHeading = None
        self.scanRotation = 0.0
        self.lastTreeOmega = -0.12

        # Apple mapping
        self.appleRadius = robotHelpers.averageAppleRadius(self)
        self.appleDiameter = self.appleRadius * 2
        self.applePositions = []
        self.tempAppleReadings = []
        self.appleMarkers = []

        # Camera
        self.fov = self.sim.getObjectFloatParam(
            self.camera,
            self.sim.visionfloatparam_perspective_angle
        )

        self.tanHalfFov = math.tan(self.fov / 2)
        self.imageWidth = 1024

        self.focalLength = (
            self.imageWidth / (2 * math.tan(self.fov / 2))
        )

        # Navigation flags
        self.april_tag_reached = False
        self.moving_around_tree = False
        self.target_distance_to_tree = None
        self.cameraTilted = False
        self.min_distance_to_april_tag = 250

        # Ultrasonic sensors
        self.usensors = []

        for i in range(16):
            sensor = self.sim.getObject(
                f"/PioneerP3DX/ultrasonicSensor[{i}]"
            )
            self.usensors.append(sensor)

    # ----------------------------------------------------------
    # START SIMULATION
    # ----------------------------------------------------------

    def start(self):

        self.sim.stopSimulation()

        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.1)

        self.sim.startSimulation()

    # ----------------------------------------------------------
    # STOP SIMULATION
    # ----------------------------------------------------------

    def stop(self):

        for marker in self.appleMarkers:
            self.sim.removeDrawingObject(marker)

        self.appleMarkers.clear()
        self.sim.stopSimulation()

    # ----------------------------------------------------------
    # MAIN ROBOT LOOP
    # ----------------------------------------------------------

    def runSelf(self, ATD):

        while True:

            # --------------------------------------------------
            # CAPTURE CAMERA DATA
            # --------------------------------------------------

            image = robotHelpers.getCameraImage(self)
            depthImage = robotHelpers.getCameraDepth(self)

            corners, ids = ATD.detect(image)

            # --------------------------------------------------
            # APPROACH APRIL TAG
            # --------------------------------------------------

            if not self.april_tag_reached:

                if corners:
                    robotHelpers.moveTowardsAprilTag(
                        self, corners, image
                    )

                else:
                    robotHelpers.moveSelf(
                        self, v=0, heading_error=0.2
                    )

            # --------------------------------------------------
            # ALIGN WITH TREE
            # --------------------------------------------------

            elif not self.treeAligned:

                robotHelpers.calibrateRobotTangentToTree(self)

            # --------------------------------------------------
            # TREE FOLLOWING AND APPLE SCANNING
            # --------------------------------------------------

            else:

                if not self.cameraTilted:
                    robotHelpers.tiltCameraX(self, 0)
                    self.cameraTilted = True

                now = time.monotonic()

                # ----------------------------------------------
                # STATIONARY SCANNING
                # ----------------------------------------------

                if self.scanState == "scanning":

                    robotHelpers.moveSelf(
                        self, v=0, heading_error=0
                    )

                    if now >= self.scanStateUntil:

                        self.scanViewpoint += 1

                        if self.scanViewpoint >= self.maxScanViewpoints:

                            self.scanState = "searching"

                        else:

                            self.scanState = "repositioning"
                            self.scanStateUntil = (
                                now + self.repositionDuration
                            )

                # ----------------------------------------------
                # MOVE TO NEXT VIEWPOINT
                # ----------------------------------------------

                elif self.scanState == "repositioning":

                    if now >= self.scanStateUntil:

                        robotHelpers.moveSelf(
                            self, v=0, heading_error=0
                        )

                        self.scanState = "scanning"
                        self.scanStateUntil = (
                            now + self.scanDuration
                        )

                    else:

                        robotHelpers.moveAroundTree(self)

                # ----------------------------------------------
                # SEARCH FOR APPLES
                # ----------------------------------------------

                else:

                    robotHelpers.moveAroundTree(self)

                # ----------------------------------------------
                # DETECT APPLES
                # ----------------------------------------------

                detections = (
                    robotHelpers.detectMarker(self, image) or []
                    if self.scanState != "repositioning"
                    else []
                )

                for (
                    boxCenter,
                    applePixelWidth,
                    (centerX, centerY),
                    (x, y, w, h)
                ) in detections:

                    section = image[y:y+h, x:x+w]

                    if section.size == 0:
                        continue

                    hsv = cv2.cvtColor(
                        section, cv2.COLOR_BGR2HSV
                    )

                    mask = cv2.inRange(
                        hsv, (0, 100, 100), (10, 255, 255)
                    )

                    mask |= cv2.inRange(
                        hsv, (160, 100, 100), (180, 255, 255)
                    )

                    ys, xs = np.where(mask > 0)

                    if len(xs) == 0:
                        continue

                    # ------------------------------------------
                    # ESTIMATE DEPTH
                    # ------------------------------------------

                    depthValues = depthImage[y + ys, x + xs]

                    appleDepth = (
                        np.median(depthValues) + self.appleRadius
                    )

                    if not np.isfinite(appleDepth):
                        continue

                    if appleDepth <= 0 or appleDepth > 4:
                        continue

                    # ------------------------------------------
                    # ESTIMATE APPLE COUNT
                    # ------------------------------------------

                    widthRatio = robotHelpers.appleWidthRatio(
                        self, appleDepth, applePixelWidth
                    )

                    if widthRatio < 0.6:
                        continue

                    estimatedAppleCount = max(
                        1, round(widthRatio)
                    )

                    # ------------------------------------------
                    # BEGIN NEW SCANNING CYCLE
                    # ------------------------------------------

                    if self.scanState == "searching":

                        self.scanState = "scanning"
                        self.scanViewpoint = 0
                        self.scanCycle += 1

                        self.scanStateUntil = (
                            time.monotonic() + self.scanDuration
                        )

                        robotHelpers.moveSelf(
                            self, v=0, heading_error=0
                        )

                        print("Starting scan cycle:", self.scanCycle)

                        # Wait for a fresh stationary camera image.
                        break

                    if self.scanState != "scanning":
                        continue

                    # ------------------------------------------
                    # MULTIPLE APPLES
                    # ------------------------------------------

                    if estimatedAppleCount > 1:

                        robotHelpers.mapMultipleApples(
                            self,
                            image,
                            depthImage,
                            x, y, w, h,
                            estimatedAppleCount
                        )

                        continue

                    # ------------------------------------------
                    # SINGLE APPLE
                    # ------------------------------------------

                    worldDirection, localDirection = (
                        robotHelpers.find3dVector(
                            self, boxCenter
                        )
                    )

                    appleCameraPosition = (
                        robotHelpers.Position3DappleRelToCamera(
                            appleDepth, localDirection
                        )
                    )

                    appleRobotPosition = (
                        robotHelpers.TransformApplePosFromCameraToRobot(
                            self, appleCameraPosition
                        )
                    )

                    appleWorldPosition = (
                        robotHelpers.TransformAppleFromRobotToWorldPos(
                            self, appleRobotPosition
                        )
                    )

                    robotHelpers.mapApple(
                        self, appleWorldPosition
                    )

                print(
                    f"State: {self.scanState} | "
                    f"Viewpoint: {self.scanViewpoint} | "
                    f"Apples mapped: {len(self.applePositions)}"
                )

            # --------------------------------------------------
            # DISPLAY CAMERA
            # --------------------------------------------------

            displayImage = cv2.aruco.drawDetectedMarkers(
                image.copy(), corners, ids
            )

            cv2.imshow(
                "robot camera", cv2.flip(displayImage, 1)
            )

            if cv2.waitKey(30) & 0xFF == ord('q'):
                break