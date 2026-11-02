from pioneerRobot import myRobot
import cv2
import time

robot = myRobot()

try: 
    robot.start()
    time.sleep(1)
    image = robot.getCameraImage()
    cv2.imshow("Pioneer Camera", image)
    cv2.waitKey(0)

finally:
    robot.stop()
    cv2.destroyAllWindows()