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

        # corners comes out looking like this 
        # which is an array of april tags (each made up of an array of corners):

        # Detected: (array([[[185., 341.],
        # [291., 341.],
        # [320., 360.],
        # [219., 359.]]], dtype=float32),)


        return corners, ids
     # ----------------------------------------------------------