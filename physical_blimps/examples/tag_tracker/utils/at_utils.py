import cv2 as cv
import numpy as np

class AprilTagHelper:
    def __init__(self, K_path, dist_path, sz=0.19):
        Kvals = np.loadtxt(K_path)
        self.K = np.eye(3)
        self.K[0,0] = Kvals[0] # fx
        self.K[1,1] = Kvals[1] # fy
        self.K[0,2] = Kvals[2] # cx
        self.K[1,2] = Kvals[3] # cy
        self.dist = np.loadtxt(dist_path)
        self.arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_APRILTAG_36h11)
        self.arucoParams = cv.aruco.DetectorParameters_create()

        self.sz = sz
        
    def detect(self, img, label=False):
        # prep image for detection
        cv_img = cv.cvtColor(np.uint8(255*img), cv.COLOR_RGB2BGR)
        gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)

        # detect tags
        (corners, ids, rejected) = cv.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)

        # parse tag info
        poses = []
        if ids is not None:            
            for (markerCorner, markerID) in zip(corners, ids):
                rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(markerCorner,
                                                                              self.sz,
                                                                              self.K,
                                                                              self.dist)
                poses.append([markerID[0], rvec, tvec.squeeze()])

        # generate labeled image (if requested)
        if label:
            if ids is not None:
                for (markerCorner, markerID) in zip(corners, ids):
                    corners = markerCorner.reshape((4,2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners

                    # convert each of the (x, y)-coordinte pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # draw the bounding box lines
                    cv.line(img, topLeft, topRight, (0, 1, 0), 2)
                    cv.line(img, topRight, bottomRight, (0, 1, 0), 2)
                    cv.line(img, bottomRight, bottomLeft, (0, 1, 0), 2)
                    cv.line(img, bottomLeft, topLeft, (0, 1, 0), 2)

                    # compute and draw the center
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv.circle(img, (cX, cY), 4, (0, 0, 1), -1)

            return poses, img

        return poses

if __name__=='__main__':
    at = AprilTagHelper("K_360x240.txt", "dist_360x240.txt")
