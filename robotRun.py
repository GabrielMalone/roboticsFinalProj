from pioneerRobot import myRobot
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

    # run camera continuously
    while True:
        # image from CoppeliaSim
        image = robot.getCameraImage()
        # show in window
        cv2.imshow("robot camera", cv2.flip(image, 0))
        corners, ids = ATD.detect(image)
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        # give 30ms for a keyboard check
        #  and check for 'q'
        # with bitwise operation
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

finally:
    robot.stop()
    cv2.destroyAllWindows()