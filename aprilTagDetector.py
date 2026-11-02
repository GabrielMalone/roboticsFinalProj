import cv2
# --------------------------------------------------------------
class AprilTagDetector:
    # ----------------------------------------------------------
    def __init__(self):
        # select the AprilTag family
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        # create detector
        self.detector = cv2.aruco.ArucoDetector(dictionary)
    # ----------------------------------------------------------
    def detect(self, image):
        # now we have an object made to detect the type of 
        # april tags that should be in our scene
        # we will have it look at what our robot cmaera can see
        corners, ids, rejected = self.detector.detectMarkers(image)

        print("Detected:", ids)
        print("Rejected candidates:", len(rejected))

        cv2.aruco.drawDetectedMarkers(
            image,
            rejected,
            borderColor=(0, 0, 255)
        )
        # let's see what the detector sees
        cv2.imwrite("rejectedMarkers.png", image)

        return corners, ids