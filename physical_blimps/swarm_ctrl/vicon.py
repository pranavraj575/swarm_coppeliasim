import numpy as np
import time


class Vicon:
    def __init__(self, num_objects):
        """
        for interface with the Vicon system
            grabs object positions and headings

        @param num_objects: number of objects registered in the vicon thingy
        """
        self.num_objects = num_objects
        self.object_data = dict()
        for obj_id in range(self.num_objects):
            self.object_data[obj_id] = {
                'time': -1,
                'linear': {'pos': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
                'angular': {'euler': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)}
            }

        # raise NotImplementedError()

    def update(self, obj_id):
        """
        updates values for obj id
        @param obj_id: object to update values for

        unimplemented, gotta grab this from vicon
            use the easy update for linear velocity, acceleration
            for angular, make sure you take the velocity/acceleration accounting for topology of circle
        """
        raise NotImplementedError()

    def get_last_update_time(self, obj_id, update=True):
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['time']

    def get_object_pos(self, obj_id, update=True):
        """
        Gets xyz position of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, xyz position of object
        """
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['linear']['pos']

    def get_object_head(self, obj_id, update=True):
        """
        Gets heading of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R, yaw angle of object from global coordinates
        """
        return self.get_object_euler(obj_id=obj_id, update=update)[2]

    def get_object_euler(self, obj_id, update=True):
        """
        Gets eulerian rpy of object specified

        @param obj_id:  0<=obj_id<self.num_objects
            index of object to return
        @return: roll, pitch, yaw
        """
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['angular']['euler']

    def get_object_vel(self, obj_id, update=True):
        """
        Gets xyz velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, xyz velocity of object
        """
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['linear']['vel']

    def get_object_angular_vel(self, obj_id, update=True):
        """
        Gets angular velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, roll, pitch, yaw velocity
        """
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['angular']['vel']

    def get_head_speed(self, obj_id, update=True):
        """
        Gets yaw velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R, yaw angular velocity
        """
        return self.get_object_angular_vel(obj_id=obj_id, update=update)[2]

    def get_object_acc(self, obj_id, update=True):
        """
        Gets xyz acceleration of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, xyz acceleration of object
        """
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['linear']['acc']

    def get_object_angular_acc(self, obj_id, update=True):
        """
        Gets angular velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, roll, pitch, yaw acceleration
        """
        if update:
            self.update(obj_id=obj_id)
        return self.object_data[obj_id]['angular']['acc']
