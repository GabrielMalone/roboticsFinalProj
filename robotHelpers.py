# ------------------------------------------------------------
#                       MOVMENT HELPERS
# --------------------------------------------------------------
def moveTowardsAprilTag(self, corners, image):
    tagCenterX= detectAprilTagCenter(self, corners)
    heading_error = detectAprilTagHeadingError(self, tagCenterX, image)
    # if the april tag is not very larg in camera, 
    # keep moving towards it
    # tagsize of 150-200 seems decent
    if tagWidth(self, corners) < self.min_distance_to_april_tag:
        # print("tag widh: ", tagWidth(self, corners))
        moveSelf(self, v=0.2, heading_error=heading_error)
    else:
        # stop
        self.april_tag_reached = True
        # print("APRIL TAG REACHED")
        moveSelf(self, v=0, heading_error=0)
# ---------------------------------------------------------
def moveAroundTree(self):
    self.moving_around_tree = True
    sensors = readUltrasonicSensors(self, "right")
    detected7, d7 = sensors[0]
    detected8, d8 = sensors[1]
    # Lost contact with tree
    if not detected7 or not detected8:
        # print("LOST SENSOR CONTACT")
        moveSelf(self, 0.1, self.lastTreeOmega)
        return
    # print("sensor data: ", d7, d8)
    # compare front and rear distances
    error = d7 - d8
    # this will be our constant clockwise rotation
    baseOmega = -0.12
    # Correct robot orientation
    omega = baseOmega - self.angular_gain * error
    self.lastTreeOmega = omega
    moveSelf(self, v=0.1, heading_error=omega)
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
    import numpy as np
    leftPos     = self.sim.getObjectPosition(self.left_motor, self.sim.handle_world)
    rightPos    = self.sim.getObjectPosition(self.right_motor, self.sim.handle_world)
    # euclidean distance
    return np.linalg.norm(np.array(leftPos) - np.array(rightPos)) 
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

# ------------------------------------------------------------
#                   TREE ALIGNBMENT HELPERS
# ------------------------------------------------------------
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
def calibrateRobotTangentToTree(self):
    import math
    # where we at right now?
    currentHeading = self.sim.getObjectOrientation(
        self.robot, -1
    )[2]

    if self.previousScanHeading is None:
        self.previousScanHeading = currentHeading
    # step 1: complete one full rotation
    if not self.treeScanComplete:
        # just move slowly 
        moveSelf(self, v=0, heading_error=0.2)
        # this records/tracks best heading 
        scanTreeAlignment(self)

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
            moveSelf(self, v=0, heading_error=0)
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
            moveSelf(self, v=0, heading_error=0)
            self.treeAligned = True
            print("TREE ALIGNMENT COMPLETE")
        # otherwise rotate towards best heading
        else:
            moveSelf(self, v=0, heading_error=headingError)
# ---------------------------------------------------------
# are we perpendicular or not?
# ---------------------------------------------------------
def scanTreeAlignment(self):
    sensors = readUltrasonicSensors(self, "right")

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
            # print(
            #     f"Best alignment: {difference:.4f} m | "
            #     f"Heading: {self.bestHeading:.3f} rad"
            # )
# ---------------------------------------------------------
# idea here is that we get close to tree with april tag
# then we do a 360 spin to see which heading the robot 
# is most perpendicular to the tree at, using the ultrasonic
def getRightDistance(self):
    readings = readUltrasonicSensors(self, "right")
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
            pass
             #print(f"Sensor {i}: {distance:.2f} m")



# ------------------------------------------------------------
#                   VISION HELPERS
# ------------------------------------------------------------

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
    import cv2
    import numpy as np
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
        boxes = []
        c = max(contours, key=cv2.contourArea)

        for c in contours:

            # simplified shape of the contour
            # we detected
            x, y, w, h = cv2.boundingRect(c)
   
            margin = 10
            if (x < margin or y < margin or
                x + w > imgw - margin or
                y + h > imgh - margin):
                continue
            
            # Draw bounding rectangle
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (255, 255, 0),
                2
            )

            # Draw arrow toward image center
            # cv2.arrowedLine(
            #     frame,
            #     (x + w // 2, y + h // 2),
            #     (imgw // 2, imgh // 2),
            #     (0, 0, 255),
            #     2
            # )
            # ok got this
            # now the quodrotor knows
            # where it needs to go to get 
            # red box back into center
            # also includes center coordinates
            boxes.append((
                transform(self, x, y, w, h, imgw, imgh),
                w,
                (int(x + w / 2), int(y + h / 2)),
                (x, y, w, h)
            ))
        return boxes
    return None
# ----------------------------------------------------------
def getCameraImage(self):
    import cv2
    import numpy as np
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
    # convert RGB to BGR for OpenCV (otherwise colors get swapped)
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

# ----------------------------------------------------------
# we know: applePixelWidth / focalLength = appleDiameter / distance
# so distance = ( appleDiameter + focal length ) / applePixelWidth

# this is the straightline forward distance to the apple 
# estiamated from the camera
def estimateAppleDistance(self, applePixelWidth):
    return (
        self.appleDiameter * self.focalLength
    ) / applePixelWidth
# ----------------------------------------------------------
def mapApple(self, appleWorldPosition):
    import numpy as np

    # --------------------------------------------------
    # MAPPING THRESHOLDS
    # --------------------------------------------------

    groupingDistance = 0.05

    # Requires more distinct viewpoints before mapping.
    minReadings = 4
    # Requires stronger evidence for potentially duplicate apples.
    ambiguousReadings = 5
    maxReadings = max(minReadings, ambiguousReadings)

    readingTolerance = 0.02
    # Treats nearby detections as existing apples rather than new ones.
    refinementDistance = 0.05
    minRefinementDistance = 0.01
    # Makes it harder to register another apple near an existing one.
    ambiguousDistance = 0.06
    # Requires more consistent position estimates.
    ambiguousTolerance = 0.015

    accuracyThreshold = 0.05
    accurateColor = [1, 1, 0]
    inaccurateColor = [0.5, 0, 1]
    boxLineWidth = 3

    # --------------------------------------------------
    # CACHE ACTUAL APPLE POSITIONS
    # --------------------------------------------------

    appleWorldPosition = np.array(appleWorldPosition)

    if not hasattr(self, "actualApplePositions"):
        self.actualApplePositions = []

        for obj in self.sim.getObjectsInTree(self.sim.handle_scene):
            name = self.sim.getObjectAlias(obj)

            if name.lower().startswith("apple"):
                self.actualApplePositions.append(
                    np.array(self.sim.getObjectPosition(
                        obj, self.sim.handle_world
                    ))
                )

    # --------------------------------------------------
    # ONLY COLLECT READINGS WHILE SCANNING
    # --------------------------------------------------

    if self.scanState != "scanning":
        return

    viewpointID = (self.scanCycle, self.scanViewpoint)

    # --------------------------------------------------
    # GROUP NEARBY READINGS BY VIEWPOINT
    # --------------------------------------------------

    for group in self.tempAppleReadings:

        average = np.mean(
            [np.mean(samples, axis=0) for samples in group.values()],
            axis=0
        )

        if np.linalg.norm(appleWorldPosition - average) < groupingDistance:

            group.setdefault(viewpointID, []).append(appleWorldPosition)

            if len(group) > maxReadings:
                del group[next(iter(group))]

            break

    else:
        self.tempAppleReadings.append({
            viewpointID: [appleWorldPosition]
        })
        return

    # --------------------------------------------------
    # REQUIRE MULTIPLE DISTINCT VIEWPOINTS
    # --------------------------------------------------

    if len(group) < minReadings:
        return

    viewpointAverages = [
        np.mean(samples, axis=0)
        for samples in group.values()
    ]

    readings = viewpointAverages[-minReadings:]
    average = np.mean(readings, axis=0)

    if not all(
        np.linalg.norm(pos - average) < readingTolerance
        for pos in readings
    ):
        return

    # --------------------------------------------------
    # FIND NEAREST MAPPED APPLE
    # --------------------------------------------------

    nearestIndex = None
    nearestDistance = float("inf")

    for i, pos in enumerate(self.applePositions):
        distance = np.linalg.norm(average - pos)

        if distance < nearestDistance:
            nearestDistance = distance
            nearestIndex = i

    updating = False

    # --------------------------------------------------
    # REFINE EXISTING APPLE
    # --------------------------------------------------

    if nearestDistance < refinementDistance:

        if nearestDistance < minRefinementDistance:
            return

        average = np.mean(
            [self.applePositions[nearestIndex], average],
            axis=0
        )

        updating = True

    # --------------------------------------------------
    # HANDLE AMBIGUOUS DETECTIONS
    # --------------------------------------------------

    elif nearestDistance < ambiguousDistance:

        if len(group) < ambiguousReadings:
            return

        readings = viewpointAverages[-ambiguousReadings:]
        average = np.mean(readings, axis=0)

        if not all(
            np.linalg.norm(pos - average) < ambiguousTolerance
            for pos in readings
        ):
            return

    # --------------------------------------------------
    # EVALUATE ACCURACY (VISUAL FEEDBACK ONLY)
    # --------------------------------------------------

    error = min(
        (np.linalg.norm(average - pos)
         for pos in self.actualApplePositions),
        default=float("inf")
    )

    color = (
        accurateColor if error < accuracyThreshold
        else inaccurateColor
    )

    if updating:
        self.sim.removeDrawingObject(
            self.appleMarkers[nearestIndex]
        )

    # --------------------------------------------------
    # CREATE BOUNDING BOX
    # --------------------------------------------------

    x, y, z = average
    s = self.appleRadius

    corners = [
        (x-s, y-s, z-s), (x+s, y-s, z-s),
        (x+s, y+s, z-s), (x-s, y+s, z-s),
        (x-s, y-s, z+s), (x+s, y-s, z+s),
        (x+s, y+s, z+s), (x-s, y+s, z+s)
    ]

    edges = [
        (0,1), (1,2), (2,3), (3,0),
        (4,5), (5,6), (6,7), (7,4),
        (0,4), (1,5), (2,6), (3,7)
    ]

    marker = self.sim.addDrawingObject(
        self.sim.drawing_lines,
        boxLineWidth,
        0,
        -1,
        len(edges),
        color
    )

    for a, b in edges:
        self.sim.addDrawingObjectItem(
            marker,
            list(corners[a]) + list(corners[b])
        )

    # --------------------------------------------------
    # UPDATE APPLE MAP
    # --------------------------------------------------

    if updating:
        self.applePositions[nearestIndex] = average
        self.appleMarkers[nearestIndex] = marker

        print("Apple position refined:", average)

    else:
        self.applePositions.append(average)
        self.appleMarkers.append(marker)

        print("Apple added to map:", average)

    # Require fresh viewpoint readings before refinement.
    group.clear()
    group[viewpointID] = [appleWorldPosition]
# ----------------------------------------------------------
def compareEstimateToActual(self, appleWorldPosition):
    apple = self.sim.getObject('/Apple')
    actualPosition = self.sim.getObjectPosition(
        apple,
        self.sim.handle_world
    )
    print("Estimated apple position:", appleWorldPosition)
    print("Actual apple position:", actualPosition)
# ----------------------------------------------------------
def TransformAppleFromRobotToWorldPos(self, appleRobotPosition):
    import numpy as np
    robotMatrix = self.sim.getObjectMatrix(self.robot, self.sim.handle_world)
    robotMatrix = np.array(robotMatrix).reshape(3, 4)
    rotation = robotMatrix[:, :3]
    translation = robotMatrix[:, 3]
    appleWorldPosition = rotation @ appleRobotPosition + translation

    return appleWorldPosition
# ----------------------------------------------------------
def TransformApplePosFromCameraToRobot(self, appleCameraPosition):
    import numpy as np
    # get camera transformation matrix relative to the robot
    cameraMatrix = self.sim.getObjectMatrix(self.camera, self.robot)
    cameraMatrix = np.array(cameraMatrix).reshape(3, 4)
    # how is the camera oriented towards the robot
    rotation = cameraMatrix[:, :3]
    # where is the camera relative to the robot
    translation = cameraMatrix[:, 3]
    # apples 3d position in meteres relative to the robot
    appleRobotPosition = rotation @ appleCameraPosition + translation
    return appleRobotPosition
# ----------------------------------------------------------
def Position3DappleRelToCamera(appleDepth, localDirection):
    import numpy as np
    appleCameraPosition = np.array([
            localDirection[0] / localDirection[2] * appleDepth,
            localDirection[1] / localDirection[2] * appleDepth,
            appleDepth
        ])
    return appleCameraPosition
# ----------------------------------------------------------
def averageAppleRadius(self):
    apple = self.sim.getObject('/Apple')
    minX = self.sim.getObjectFloatParam(
        apple, self.sim.objfloatparam_objbbox_min_x
    )
    maxX = self.sim.getObjectFloatParam(
        apple, self.sim.objfloatparam_objbbox_max_x
    )
    return (maxX - minX) / 2
# ----------------------------------------------------------
def appleWidthRatio(self, appleDepth, applePixelWidth):
    expectedWidth = (
            self.appleDiameter * self.focalLength
        ) / appleDepth
    widthRatio = applePixelWidth / expectedWidth
    # print(f"Expected: {expectedWidth:.1f} px")
    return widthRatio
# ----------------------------------------------------------
def find3dVector(self, boxCenter):
    import numpy as np
    # The apple appears halfway between the center and right edge of my image (x=0.5).
    # Since my camera has a 60° field of view,
    # I know the right edge corresponds to a 30° angle from straight ahead.
    # At a forward distance of 1 meter, that edge is 0.577 meters to the right.
    # Since my apple is halfway toward that edge,
    # its position would be 0.2887 meters to the right at that same forward distance.
    x = -boxCenter[0] * self.tanHalfFov
    y = boxCenter[1] * self.tanHalfFov
    z = 1.0
    # now we have a forward diretion vector
    # pointing toward apple from camera
    direction = np.array([x, y, z])
    # unit vector pointing to apple from camera
    direction = direction / np.linalg.norm(direction)

    #get camera world rotation
    # we need to extract the 3x3 rotation matrix
    # from this

    cameraMatrix = self.sim.getObjectMatrix(
        self.camera,
        self.sim.handle_world
    )
    # print(f"cameraMatrix: {cameraMatrix}")
    matrix = np.array(cameraMatrix).reshape(3, 4)
    rotation = matrix[:, :3]
    # @ = matrix multiplication in numpy
    worldDirection = rotation @ direction
    # so now we have a 3dvector pointing towards the apple
    # and its coordinates are rotated to world space

    # how to get distance at this point?
    return worldDirection, direction
# ----------------------------------------------------------
def getCameraDepth(self):
    import numpy as np
    # depth is a flat array of depth data per pixel (32bit floats)
    depth, resolution = self.sim.getVisionSensorDepth(
        self.camera, 1
    )
    # turn depth into a 2d array where we can access depth data
    # via x y coords e.g. depth[320, 200] = 1.7 meters
    depth = np.frombuffer(depth, dtype=np.float32)
    depth = depth.reshape(resolution[1], resolution[0])
    # print("Depth type:", type(depth))
    # print("Resolution:", resolution)

    return depth
# ----------------------------------------------------------
def tiltCameraX(self, degrees):
    import math
    self.sim.setObjectOrientation(
        self.camera,
        self.robot,
        [math.radians(degrees),0,0]
    )
# ----------------------------------------------------------
def tiltCameraY(self, degrees):
    import math
    self.sim.setObjectOrientation(
        self.camera,
        self.robot,
        [0,math.radians(degrees),0]
    )
# ----------------------------------------------------------
def tagWidth(self, corners):
    import numpy as np
    tagCorners = corners[0][0]
    return np.linalg.norm(tagCorners[1] - tagCorners[0])
# ----------------------------------------------------------
def detectAprilTagCenter(self, corners):
    import numpy as np
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
def removeMergedApple(self, mergedWorldPosition):
    import numpy as np
    mergedWorldPosition = np.array(mergedWorldPosition)

    for i in range(len(self.applePositions) - 1, -1, -1):
        if np.linalg.norm(self.applePositions[i] - mergedWorldPosition) < 0.06:
            self.applePositions.pop(i)

            self.sim.removeDrawingObject(self.appleMarkers.pop(i))

            print("Removed merged apple from map")
# ---------------------------------------------------------
def mapMultipleApples(self, image, depthImage, x, y, w, h, estimatedAppleCount):
    import numpy as np
    import cv2
    sectionWidth = w / estimatedAppleCount

    for i in range(estimatedAppleCount):

        sectionLeft = int(x + i * sectionWidth)
        sectionRight = int(x + (i + 1) * sectionWidth)

        section = image[y:y+h, sectionLeft:sectionRight]
        hsv = cv2.cvtColor(section, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask |= cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))

        ys, xs = np.where(mask > 0)

        if len(xs) == 0:
            continue

        appleCenterX = sectionLeft + int(np.median(xs))
        appleCenterY = y + int(np.median(ys))

        print(f"Apple {i + 1} center: ({appleCenterX}, {appleCenterY})")

        # Normalize individual apple center
        normalizedX = (appleCenterX / self.imageWidth) * 2 - 1
        normalizedY = (appleCenterY / depthImage.shape[0]) * 2 - 1

        appleBoxCenter = (normalizedX, normalizedY)

        # Get depth for individual apple
        appleDepth = np.median(
            depthImage[y + ys, sectionLeft + xs]
        ) + self.appleRadius

        print("Depth:", appleDepth, "Pixel BGR:", image[appleCenterY, appleCenterX])
        print(f"Apple {i+1} depth: {appleDepth:.3f}")

        if appleDepth > 2.0:
            continue

        # Calculate 3D direction
        worldDirection, localDirection = find3dVector(
            self, appleBoxCenter
        )

        # Camera -> Robot -> World
        appleCameraPosition = Position3DappleRelToCamera(
            appleDepth, localDirection
        )

        appleRobotPosition = TransformApplePosFromCameraToRobot(
            self, appleCameraPosition
        )

        appleWorldPosition = TransformAppleFromRobotToWorldPos(
            self, appleRobotPosition
        )

        print(f"Apple {i + 1} world position: {appleWorldPosition}")

        mapApple(self, appleWorldPosition)