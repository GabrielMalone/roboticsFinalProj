import cv2

# select the AprilTag 36h11 family
# Augmented Reality University of Córdoba 
# (aruco -- where april tags invented)
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

# generate 400x400 AprilTag with ID 0
tag = cv2.aruco.generateImageMarker(
    dictionary,
    0,
    400
)

# save image
cv2.imwrite("aprilTag0.png", tag)