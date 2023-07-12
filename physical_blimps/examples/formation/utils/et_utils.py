import cv2 as cv
import numpy as np

class EllHelper:
    def __init__(self, parampath="decay_params.txt"):
        # ellipse detector params
        self.params = cv.SimpleBlobDetector_Params()

        self.params.filterByArea = True
        self.params.minArea = 1500
        self.params.maxArea = 100000000

        self.params.filterByConvexity = False
        self.params.minConvexity = 0.2

        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.0
        self.params.maxInertiaRatio = 0.6
        
        self.detector = cv.SimpleBlobDetector_create(self.params)

        # hsv filtering params
        self.low_hsv = np.array([88,115,75])
        self.hi_hsv = np.array([107,255,230])

        # morphology params
        self.open_kernel = np.ones((3,3), np.uint8)
        self.close_kernel = np.ones((15,15), np.uint8)
        self.dilate_kernel = np.ones((3,3), np.uint8)
        self.dilate_its = 4
        self.median_kernel_sz = 7
        
        # contour ellipse params
        self.minArea = 50.
        self.params = np.loadtxt(parampath)
        self.xpixels2rads = (62.2/360.)*(np.pi/180)
        self.ypixels2rads = (48.8/240.)*(np.pi/180)

        # boundary criteria
        self.bound_range = 5

        # raspberry pi camera v2 has 
        # - fov of 62.2 x 48.8 degrees
        # - image size of 3280 x 2464
        # - if binned, multiply by 2
        
    def detect(self, img_, label=False):
        img = img_.copy()
        
        # prep image for detection
        cv_img = cv.cvtColor(np.uint8(255*img), cv.COLOR_RGB2BGR)
        hsv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)

        # filter pixels with HSV
        mask = cv.inRange(hsv_img, self.low_hsv, self.hi_hsv)
        
        # apply morphological operators on mask
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.open_kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.close_kernel)
        mask = cv.dilate(mask, self.dilate_kernel, self.dilate_its)
        
        # smooth mask with median filter
        mask = cv.medianBlur(mask, self.median_kernel_sz)

        # find contours
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # find biggest ellipse
        ell, ell_a = None, 0.
        cX, cY = None, None
        if len(contours) > 0:
            for i, c in enumerate(contours):
                if c.shape[0] > 5:
                    M = cv.moments(c)
                    area = M['m00']

                    # filter by area
                    if area < self.minArea or area < ell_a: continue
    
                    # filter if skinny
                    xdis = np.max(c[:,:,0])-np.min(c[:,:,0])
                    ydis = np.max(c[:,:,1])-np.min(c[:,:,1])
                    if ydis > xdis: continue

                    # filter if close to border
                    if (c[:,:,0] < self.bound_range).any(): continue
                    if (c[:,:,1] < self.bound_range).any(): continue
                    if (360-c[:,:,0] < self.bound_range).any(): continue
                    if (240-c[:,:,1] < self.bound_range).any(): continue

                    # add as a valid contour
                    ell, ell_a = cv.fitEllipse(c), area
                    cX, cY = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

        # find the distance the ellipse is away
        p = 3*[None]
        if ell is not None:
            dis = self.area2dis(ell_a)

            # compute the relative 3D position (in blimp coordinate frame)
            cX_ = int(img.shape[1]/2) - cX
            cY_ = int(img.shape[0]/2) - cY

            # convert to polar coordinates
            aZ = self.ypixels2rads*cX_ - 0.3
            aY = self.xpixels2rads*cY_

            # convert to position
            p[0] = dis*np.cos(aZ)*np.cos(aY)
            p[1] = dis*np.sin(aZ)
            p[2] = dis*np.sin(aY)


        # highlight detected blimp in image (optional)
        if label:
            # draw ellipse on image
            img_masked = cv_img.copy()            
            if ell is not None: cv.ellipse(img_masked, ell, (0,255,0), thickness=1)
            
            # draw centroid
            if ell is not None: 
                r = int(12./dis)
                cv.circle(img_masked, (cX, cY), r, (0,255,0), -1)

            return np.array(p), cv.cvtColor(img_masked, cv.COLOR_BGR2RGB)/255.

        return np.array(p)
        
    def area2dis(self, A):
        # convert to distance
        return self.params[0] * ((A/1000.) ** self.params[1])

