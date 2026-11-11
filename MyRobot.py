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
        self.wheel_radius   = self.getWheelRadius()
        self.L              = self.getWheelLength()
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

        # other flags
        self.april_tag_reached = False
        self.moving_around_tree = False
        self.target_distance_to_tree = None
        self.cameraTilted = False

        # actually the x and y top corner 
        # length of april tag in pixels
        self.min_distance_to_april_tag = 250

        # get the 16 ultrasonic sensors
        self.usensors = []
        for i in range(16):
            sensor = self.sim.getObject(
                f"/PioneerP3DX/ultrasonicSensor[{i}]"
            )
            self.usensors.append(sensor)
    # ---------------------------------------------------------- 
    def start(self):
        self.sim.startSimulation()
    # ----------------------------------------------------------
    def runSelf(self, ATD):
        # run camera continuously
        while True:
            # --------------------------------------------------
            # image from CoppeliaSim
            # --------------------------------------------------
            image = self.getCameraImage()
            # any april tags in view?
            corners, ids = ATD.detect(image)
            # for visualization in the sim
            image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
            # --------------------------------------------------
            # if april tag detected move towards it
            # --------------------------------------------------
            if (corners and not self.april_tag_reached):
                self.moveTowardsAprilTag(corners, image)
            # --------------------------------------------------
            # otherwise spin in a circle
            # and look for tag
            # --------------------------------------------------
            elif not self.april_tag_reached:
                print("just going in circles!")
                self.moveSelf(v=0, heading_error=0.2)
            # --------------------------------------------------
            # tree / robot alignment 
            # --------------------------------------------------
            if self.april_tag_reached and not self.treeAligned:
                self.calibrateRobotTangentToTree()
            # --------------------------------------------------
            # once aligned, we can move around the tree
            # --------------------------------------------------
            if self.treeAligned:
                self.moveAroundTree()
                if not self.cameraTilted:
                    self.tiltCameraX(0)
                    self.cameraTilted = True
                boxCenter = self.detectMarker(image)
                if boxCenter is not None:
                    print(boxCenter)
                    
            cv2.imshow("robot camera", cv2.flip(image, 1))
            # --------------------------------------------------
            # keyborad input stuff
            # --------------------------------------------------
            # give 30ms for a keyboard check
            # and check for 'q' to quit
            # with bitwise operation
            
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
    # ----------------------------------------------------------
    def tiltCameraX(self, degrees):
        self.sim.setObjectOrientation(
            self.camera,
            self.robot,
            [math.radians(degrees),0,0]
        )
    # ----------------------------------------------------------
    def tiltCameraY(self, degrees):
        self.sim.setObjectOrientation(
            self.camera,
            self.robot,
            [0,math.radians(degrees),0]
        )
    # ----------------------------------------------------------
    def getTreeRadius(self):
        tree = self.sim.getObject('/Tree')
        minX = self.sim.getObjectFloatParam(
            tree, self.sim.objfloatparam_objbbox_min_x
        )
        maxX = self.sim.getObjectFloatParam(
            tree, self.sim.objfloatparam_objbbox_max_x
        )
        treeRadius = (maxX - minX) / 2
        return treeRadius
    # ----------------------------------------------------------
    def getWheelRadius(self):
        wheel    = self.sim.getObject('/PioneerP3DX/rightMotor/rightWheel')
        minX     = self.sim.getObjectFloatParam(
            wheel, self.sim.objfloatparam_objbbox_min_x
        )
        maxX     = self.sim.getObjectFloatParam(
            wheel, self.sim.objfloatparam_objbbox_max_x
        )
        return (maxX - minX) / 2
    # ----------------------------------------------------------
    def getWheelLength(self):
        leftPos     = self.sim.getObjectPosition(self.left_motor, self.sim.handle_world)
        rightPos    = self.sim.getObjectPosition(self.right_motor, self.sim.handle_world)
        # euclidean distance
        return np.linalg.norm(np.array(leftPos) - np.array(rightPos))
    # ----------------------------------------------------------
    def stop(self):
        self.sim.stopSimulation()
    # ----------------------------------------------------------
    def moveSelf(self, v, heading_error):

        omega = self.heading_gain * heading_error

        if (self.moving_around_tree):
            # Clamps the value to the specified range
            omega = heading_error
            
        # move forward while steering
        # convert forward and angular velocity into wheel velocities
        # v = wr and L/2 = r for turning angular into forward v
        left_velocity   = v - ( omega * (self.L / 2) ) 
        right_velocity  = v + ( omega * (self.L / 2) )
        # Convert linear wheel velocities into radians/second
        # just rearranging previous equation : w = v/r
        phi_hat_left    = left_velocity / self.wheel_radius
        phi_hat_right   = right_velocity / self.wheel_radius
        # Send commands to CoppeliaSim
        self.sim.setJointTargetVelocity(self.left_motor, phi_hat_left)
        self.sim.setJointTargetVelocity(self.right_motor, phi_hat_right)
    # ----------------------------------------------------------
    def getCameraImage(self):
        # get image from coppeliasim
        # resolution [1024, 1024]
        image, resolution = self.sim.getVisionSensorImg(self.camera)
        # print("resolution: ", resolution)
        # coppeliasim sends the image as continuous sequence of bytes
        # 255, 0, 124, 0, 255, etc...
        # to turn this into an array of 8 byte units (0,255)
        # [255, 0, 124, 0, 255, etc...]
        image = np.frombuffer(image, dtype=np.uint8)
        # reshape into a 3d array: 256x256 rows cols 
        # containing a 3 item array [r,g, b]
        image = image.reshape(resolution[1], resolution[0], 3)
        # have to flip the image for some reason (and sometimes not?!)
        # image = cv2.flip(image, 0)
        # convert RGB to BGR for OpenCV (otherwise colors get swapped)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # ----------------------------------------------------------
    def tagWidth(self, corners):
        tagCorners = corners[0][0]
        return np.linalg.norm(tagCorners[1] - tagCorners[0])
    # ----------------------------------------------------------
    def findAprilTag(self):
        # if no april tag in view, just turn until find one
        pass
    # ----------------------------------------------------------
    def detectAprilTagCenter(self, corners):

        # we can get the pixel coordinates of the four conrers of
        # the april tag. these coordinates will be compared against
        # the dimensions of our camera. that way we can tell if
        # the tag is to the left/right of center. 

        # just get the first tag detected for now

        tagCorners = corners[0][0]
        # corners[0][0] looks like

        # first tag's corners (x,y coordinats in camera):  
        # [
        #    [186. 341.]
        #    [299. 342.]
        #    [320. 360.]
        #    [219. 359.]
        # ]

        tagCenterX = np.mean(tagCorners[:, 0])
        return tagCenterX
    
   # ----------------------------------------------------------
    def detectAprilTagHeadingError(self, tagCenterX, image):
        # average the horizontal position of the four corners
        # seelct every row : and just the first column 0 (so every x value)
        
        # get width of camera image and / 2 for x center of the camera 
        imageCenterX = image.shape[1] / 2

        # if tag is at 150 pixels and image center at 250 pixels 
        # then we know tag to the left of center 
        # divide by camera size to normize 
        # (so we get value indpendent of camera res)
        headingError = (tagCenterX - imageCenterX) / imageCenterX  

        return headingError  
    # ---------------------------------------------------------
    def readUltrasonicSensors(self, side=""):
        # looks like we are about 0.68 meters away 
        # from tree when finish approaching april tag

        if side == "right":
            # _, not using the last three values returned
            # sensors 6 and 7 are the right side sensors
            # first value r1 just a boolean yes / no somehting detected
            # second d1 is how far away detected object is
            r1, d1, *_ = self.sim.readProximitySensor(self.usensors[7])
            r2, d2, *_ = self.sim.readProximitySensor(self.usensors[8])
            return [(r1, d1),(r2,d2)]
        
        for i, sensor in enumerate(self.usensors): 
            result, distance, *_ = (
                self.sim.readProximitySensor(sensor)
            )
            if result:
                print(f"Sensor {i}: {distance:.2f} m")
    # ---------------------------------------------------------
    def getRightDistance(self):
        readings = self.readUltrasonicSensors("right")
        # some list comprehension
        distances = [
            distance
            for detected, distance in readings
            if detected
        ]
        # nothing on the right sensors
        if not distances:
            return None
        # whichever is closest is more valuable info
        return min(distances)
    # ---------------------------------------------------------
    def moveTowardsAprilTag(self, corners, image):
        tagCenterX= self.detectAprilTagCenter(corners)
        heading_error = self.detectAprilTagHeadingError(tagCenterX, image)
        # if the april tag is not very larg in camera, 
        # keep moving towards it
        # tagsize of 150-200 seems decent
        if self.tagWidth(corners) < self.min_distance_to_april_tag:
            print("tag widh: ", self.tagWidth(corners))
            self.moveSelf(v=0.2, heading_error=heading_error)
        else:
            # stop
            self.april_tag_reached = True
            print("APRIL TAG REACHED")
            self.moveSelf(v=0, heading_error=0)
    # ---------------------------------------------------------
    # idea here is that we get close to tree with april tag
    # then we do a 360 spin to see which heading the robot 
    # is most perpendicular to the tree at, using the ultrasonic
    # ---------------------------------------------------------
    def calibrateRobotTangentToTree(self):

        # where we at right now?
        currentHeading = self.sim.getObjectOrientation(
            self.robot, -1
        )[2]

        if self.previousScanHeading is None:
            self.previousScanHeading = currentHeading

        # step 1: complete one full rotation
        if not self.treeScanComplete:
            # just move slowly 
            self.moveSelf(v=0, heading_error=0.2)
            # this records/tracks best heading 
            self.scanTreeAlignment()

            # since we are trying to record one robot rotation
            # use atan2 to account for wrapping around pi
            # so we dont accidentally think we made a big rotation
            # when we simply crossed a +/- pi boundary

            # tells us the angle pointing towards the points we pass in
            delta = math.atan2(
                # vertical coord (y) from radians
                math.sin(currentHeading - self.previousScanHeading),
                # horizontal coord (x) from randians
                math.cos(currentHeading - self.previousScanHeading)
            )
            self.scanRotation += abs(delta)
            print(f"calibrating tree...{(self.scanRotation/ (2 * math.pi) * 100)}% complete")
            # update for next round
            self.previousScanHeading = currentHeading

            if self.scanRotation >= 2 * math.pi:
                self.treeScanComplete = True
                # scan done, stop
                self.moveSelf(v=0, heading_error=0)
                print("SCAN COMPLETE")
                print("Best heading:", self.bestHeading)

        # step 2, move robot into the best heading position
        elif self.bestHeading is not None:
            
            headingError = math.atan2(
                math.sin(self.bestHeading - currentHeading),
                math.cos(self.bestHeading - currentHeading)
            )
            # if in position
            if abs(headingError) < 0.02:
                self.moveSelf(v=0, heading_error=0)
                self.treeAligned = True
                print("TREE ALIGNMENT COMPLETE")
            # otherwise rotate towards best heading
            else:
                self.moveSelf(v=0, heading_error=headingError)

    # ---------------------------------------------------------
    # are we perpendicular or not?
    # ---------------------------------------------------------
    def scanTreeAlignment(self):
        sensors = self.readUltrasonicSensors("right")

        detected7, distance7 = sensors[0]
        detected8, distance8 = sensors[1]

        if detected7 and detected8:
            # distance measurement differences 
            # between right side detectors
            difference = abs(distance7 - distance8)
            # we are looking for similar distances
            # since that should indicated perpendicularity
            if difference < self.bestDifference:
                self.bestDifference = difference
                # record the heading of the robot when perpendicular
                self.bestHeading = self.sim.getObjectOrientation(
                    self.robot, -1
                )[2]
                print(
                    f"Best alignment: {difference:.4f} m | "
                    f"Heading: {self.bestHeading:.3f} rad"
                )
    # ---------------------------------------------------------
    def moveAroundTree(self):
        self.moving_around_tree = True

        sensors = self.readUltrasonicSensors("right")

        detected7, d7 = sensors[0]
        detected8, d8 = sensors[1]

        # Lost contact with tree
        if not detected7 or not detected8:
            print("LOST SENSOR CONTACT")
            self.moveSelf(0.1, self.lastTreeOmega)
            return

        print("sensor data: ", d7, d8)

        # compare front and rear distances
        error = d7 - d8

        # this will be our constant clockwise rotation
        baseOmega = -0.12

        # Correct robot orientation
        omega = baseOmega - self.angular_gain * error
        self.lastTreeOmega = omega

        self.moveSelf(v=0.1, heading_error=omega)
    # ---------------------------------------------------------
    def transform(self, x, y, w, h, imgw, imgh):
        # Goal: Normalize rectangle coordinates to -1 ... +1

        # x and y represent the top left corner of the bounding box
        # (x,y) of that edge
        # centerX = x + w / 2
        # centerY = y + h / 2

        # but just knowing that the center of the box
        # is at some pixel coordinate isnt very useful
        # we want to normalize the coordinate 
        # so we can generally say that the box 
        # is to the the left or right of the robot etc

        # so convert coordinates into a range from -1 to 1
        # normalizedX = (centerX/imgw) <-- now x is between 0 and 1
        # normalizedX *= 2 <-- now between 0 and 2
        # normalizedX -= 1 <-- now between -1 and 1

        centerX = x + w / 2
        centerY = y + h / 2

        normalXCenter = centerX/imgw * 2 - 1
        normalYCenter = centerY/imgh * 2 - 1

        return normalXCenter, normalYCenter
    # ----------------------------------------------------------
    def detectMarker(self, frame):
        """
        Detect red markers in the image and return their
        positions and sizes.
        """

        # Convert RGB to BGR, then HSV
        # hsv allows for easier detection of specific colors
        # regardless of lighting
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color ranges
        # red falls on two ends of the HSV spectrum
        red_ranges = [
            (
                # hue(color), saturation (vividness), value (intensity)
                np.array([0, 100, 100]),
                np.array([10, 255, 255])
            ),
            (
                # so both ends of red 0-10, 168-180 at dim->bright
                np.array([160, 100, 100]),
                np.array([180, 255, 255])
            )
        ]

        # Combine masks
        # lower = that first  array 0
        # upper = second array 10, etc

        # pass in the hsv image created above
        # Look at every pixel in hsv_image and tell me 
        # whether its HSV values fall between lower and upper.
        # summing to combine both ranges (if valid in one range, white represented)

        # open cv in range returns 255 
        # when true 0 when false (black and white)
        # this loop runs twice
        # one for each red range
        # then sums each mask for one final mask
        red_mask = sum(
            cv2.inRange(hsv_image, lower, upper)
            for lower, upper in red_ranges
        )

        # find the boundaries of the red regions
        # we just found
        contours, _ = cv2.findContours(
            red_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # contours = (
        #     array_of_points_for_red_object_1,
        #     array_of_points_for_red_object_2,
        #     array_of_points_for_red_object_3
        # )
        # Each contour contains coordinates (x, y) 
        # describing its outline.

        if contours:
            # imgh, w are dimensions of the camera output
            imgh, imgw, imgc = frame.shape

            # Find largest contour
            # key tells max function how to calculate 
            # the value being compared

            # we assume the largest countour area
            # is the box we are looking for
            c = max(contours, key=cv2.contourArea)

            # simplified shape of the contour
            # we detected
            x, y, w, h = cv2.boundingRect(c)

            # Draw bounding rectangle
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            # Draw arrow toward image center
            cv2.arrowedLine(
                frame,
                (x + w // 2, y + h // 2),
                (imgw // 2, imgh // 2),
                (0, 0, 255),
                2
            )
            # ok got this
            # now the quodrotor knows
            # where it needs to go to get 
            # red box back into center
            return self.transform(
                x, y, w, h, imgw, imgh
            )

        return None
    # ----------------------------------------------------------