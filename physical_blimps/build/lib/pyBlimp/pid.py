import time
import numpy as np
from numpy import pi

class PID:
    def __init__(self, k, angle=False, windup=None):
        self.k = k # tuning params
        self.angle = angle # if this is a PID controller for angles

        # setup PID information
        self.windup = windup
        self.e0 = 0.
        self.ei = 0.
        self.t0 = time.time()
        
    def input(self, x, x_des, given_velocity=False):
        dt = time.time() - self.t0

        # compute error terms
        e = x_des - x
        
        if given_velocity:
            # wrap the error if angle tracking
            if self.angle: e[0] = e[0] - 2*pi*(e[0] > pi) + 2*pi*(e[0] < -pi)

            # compute remaining terms
            ei = self.ei + e[0]*dt
            ed = e[1]
            
            # prevent windup
            if self.windup is not None:
                ei = max(min(ei, abs(self.windup)), -abs(self.windup))
                
            # compute the input
            u = self.k @ np.array([e[0], ei, ed])

            # store terms for next iteration
            self.e0 = e
            self.t0 = time.time()

        else:
            # wrap the error if angle tracking
            if self.angle: e = e - 2*pi*(e > pi) + 2*pi*(e < -pi)

            # compute remaining terms
            ed = (e - self.e0)/dt
            ei = self.ei + e*dt

            # prevent windup
            if self.windup is not None:
                ei = max(min(ei, abs(self.windup)), -abs(self.windup))
                
            # compute the input
            u = self.k @ np.array([e, ei, ed])

            # store terms for next iteration
            self.e0 = e
            self.t0 = time.time()

        return u
