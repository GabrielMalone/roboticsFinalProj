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

        self.april_tag_reached = False


        # actually the x and y top corner 
        # length of april tag in pixels
        self.min_distance_to_april_tag = 200

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
        # Limit angular velocity
        omega = np.clip(omega, -0.15, 0.15)

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
    def readUltrasonicSensors(self):
        # looks like we are about 0.68 meters away 
        # from tree when finish approaching april tag
        for i, sensor in enumerate(self.usensors): 
            result, distance, _, _, _ = (
                self.sim.readProximitySensor(sensor)
            )
            if result:
                print(f"Sensor {i}: {distance:.2f} m")
    # ---------------------------------------------------------
    def runSelf(self, ATD):
        # run camera continuously
        while True:

            # image from CoppeliaSim
            image = self.getCameraImage()
  
            # any april tags in view?
            corners, ids = ATD.detect(image)

            # if april tag detected move towards it
            if (corners and not self.april_tag_reached):
                tagCenterX= self.detectAprilTagCenter(corners)
                heading_error = self.detectAprilTagHeadingError(tagCenterX, image)
                # if the april tag is not very larg in camera, 
                # keep moving towards it
                # tagsize of 150-200 seems decent
                if self.tagWidth(corners) < 200:
                    print("tag widh: ", self.tagWidth(corners))
                    self.moveSelf(v=0.2, heading_error=heading_error)
                else:
                    # stop
                    self.april_tag_reached = True
                    print("APRIL TAG REACHED")
                    self.moveSelf(v=0, heading_error=0)

            # otherwise spin in a circle
            elif not self.april_tag_reached:
                self.moveSelf(v=0, heading_error=0.2)
                
            # for visualization in the sim
            image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
            cv2.imshow("robot camera", image)
      

            # give 30ms for a keyboard check
            # and check for 'q' to quit
            # with bitwise operation
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
    # ----------------------------------------------------------





