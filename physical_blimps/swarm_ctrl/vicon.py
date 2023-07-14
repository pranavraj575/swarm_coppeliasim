import numpy as np

class Vicon:
    def __init__(self):
        """
        for interface with the Vicon system
            grabs object positions and headings
        """
        self.num_objects=0

        #raise NotImplementedError()
    def get_object_pos(self,obj_id):
        """
        Gets xyz position of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, xyz position of object
        """
        return 0
        #raise NotImplementedError()
    def get_object_head(self,obj_id):
        """
        Gets heading of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R, angle of object from global x axis
        """
        return 0
        #raise NotImplementedError()