from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2 as cv
import math

# --------------------------------------------------------------
# Quadcopter
# --------------------------------------------------------------
class myRobot():

    # ----------------------------------------------------------
    def __init__(self):
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        # Get quadcopter components
        self.drone = self.sim.getObject('/Quadcopter')
        self.camera = self.sim.getObject('/Quadcopter/visionSensor')
        self.detector = self.sim.getObject('/detector')

        print("Connected from VS Code!")

    # ----------------------------------------------------------
    def readFrame(self):
        self.sim.handleVisionSensor(self.camera)

        buf, res = self.sim.getVisionSensorImg(self.camera)

        frame = np.frombuffer(buf, dtype=np.uint8)
        frame = frame.reshape(res[1], res[0], 3)

        return frame.copy()

    # ----------------------------------------------------------
    def showImage(self, img):
        self.sim.setVisionSensorImg(
            self.detector,
            img.tobytes()
        )

    # ----------------------------------------------------------
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
        image = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

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
            cv.inRange(hsv_image, lower, upper)
            for lower, upper in red_ranges
        )

        # find the boundaries of the red regions
        # we just found
        contours, _ = cv.findContours(
            red_mask,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE
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
            c = max(contours, key=cv.contourArea)

            # simplified shape of the contour
            # we detected
            x, y, w, h = cv.boundingRect(c)

            # Draw bounding rectangle
            cv.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            # Draw arrow toward image center
            cv.arrowedLine(
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
    def run(self):

        self.sim.startSimulation()

        try:
            while (
                self.sim.getSimulationState()
                != self.sim.simulation_stopped
            ):

                frame = self.readFrame()

                boxCenter = self.detectMarker(frame)

                self.showImage(frame)

                if boxCenter is not None:
                    print(boxCenter)

        except KeyboardInterrupt:
            print("Stopping simulation...")

        finally:
            self.sim.stopSimulation()


# --------------------------------------------------------------
# Main
# --------------------------------------------------------------
robot = myRobot()
robot.run()