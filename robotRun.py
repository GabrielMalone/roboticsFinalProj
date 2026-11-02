from MyRobot import myRobot
from aprilTagDetector import AprilTagDetector
import cv2
import time
# --------------------------------------------------------------
robot = myRobot()
# --------------------------------------------------------------
try: 
    robot.start()
    # give the camera a chance to get the image
    time.sleep(1)
    # load up the detector
    ATD = AprilTagDetector()
    # give the robot the April Tag Detector
    robot.runSelf(ATD)
    
finally:
    robot.stop()
    cv2.destroyAllWindows()