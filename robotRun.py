from pioneerRobot import myRobot
import cv2
import time

robot = myRobot()

try: 
    robot.start()
    # give the camera a chance to get the image
    time.sleep(1)
    image = robot.getCameraImage()

    cv2.imshow("robot camera", image)
    # stop sim with key 0
    cv2.waitKey(0)

finally:
    robot.stop()
    cv2.destroyAllWindows()