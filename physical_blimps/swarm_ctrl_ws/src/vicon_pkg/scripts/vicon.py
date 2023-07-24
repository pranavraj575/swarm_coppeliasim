#!/usr/bin/env python3
import numpy as np
import time
from collections import deque
import rospy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class Vicon:
    def __init__(self, topics,topic_global='/vicon/',queue_len=10):
        """
        for interface with the Vicon system
            grabs object positions and headings

        """
        self.num_objects = len(topics)
        self.blimps=[]
        rospy.init_node('listener_hardly_know_her')
        for obj_id,local in enumerate(topics):
            this_blimp=dict()
            object_info={
                'time': -1,
                'linear': {'pos': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
                'angular': {'euler': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
                'pos_hist':deque(maxlen=queue_len),
                'ang_hist':deque(maxlen=queue_len),
                'time_hist':deque(maxlen=queue_len),
            }
            this_blimp['info']=object_info
            topic=topic_global+local
            this_blimp['topic']=topic
            self.blimps.append(this_blimp)
            callback=self.create_subscriber_callback(dictionary=this_blimp,key='info')
            rospy.Subscriber(local.replace('/','_'),TransformStamped,callback)
        self.topics=[]
        for obj_id,local in enumerate(topics):
            topic=topic_global+local
            self.topics.append(topic)

        # raise NotImplementedError()
    def create_subscriber_callback(self,dictionary,key='info'):
        """
        creates a callback that adds to history of dictionary[key]

        @param dictionary: dictionary to put into
        @param key: key to put into
        """

        def callback(msg):
            pos=msg.transform.translation
            quat=msg.transform.rotation
            time=msg.header.stamp.secs+msg.header.stamp.nsecs*10**(-9)
            skip=False
            if dictionary[key]['time_hist']:
                last_time=dictionary[key]['time_hist'][-1]
                if time<=last_time:
                    skip=True
                    if time<last_time:
                        raise Exception("how did we get here")
                    #return
            if not skip:
                pos=np.array((pos.x,pos.y,pos.z))
                r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
                arr=r.as_euler('xyz', degrees=False)
                dictionary[key]['pos_hist'].append(pos)
                dictionary[key]['ang_hist'].append(arr)
                dictionary[key]['time_hist'].append(time)
        return callback



    def update(self, obj_ids, pos_samples=1,vel_samples=1,acc_samples=1):
        """
        updates values for obj ids
        @param obj_ids: array of object ids to update values for

        unimplemented, gotta grab this from vicon
            use the easy update for linear velocity, acceleration
            for angular, make sure you take the velocity/acceleration accounting for topology of circle
        """
        for obj_id in obj_ids:
            object_info=self.blimps[obj_id]['info'].copy()
            if not object_info['time_hist']:
                print("WARNING: trying to access unpublished topic:",self.blimps[obj_id]['topic'])
                continue
            self.blimps[obj_id]['info']['time']=object_info['time_hist'][-1]
            pos_samp=max(pos_samples,len(object_info['pos_hist']))
            self.blimps[obj_id]['info']['linear']['pos']=np.mean([object_info['pos_hist'][-i] for i in range(1,pos_samp+1)],axis=0)
            
            # TODO: should we use angle averages
            self.blimps[obj_id]['info']['angular']['euler']=object_info['ang_hist'][-1] 
            
            if len(object_info['time_hist'])>1 and vel_samples>0:
                vel_samp=max(vel_samples,len(object_info['time_hist'])-1)
                self.blimps[obj_id]['info']['linear']['vel']=np.mean([
                                                                (object_info['pos_hist'][-i]-object_info['pos_hist'][-i-1])/(object_info['time_hist'][-i]-object_info['time_hist'][-i-1]) 
                                                                for i in range(1,vel_samp+1)],axis=0)
                self.blimps[obj_id]['info']['angular']['vel']=(object_info['ang_hist'][-1]-object_info['ang_hist'][-2])/(object_info['time_hist'][-1]-object_info['time_hist'][-2])
            if len(object_info['time_hist'])>2 and acc_samples>0:
                acc_samp=max(acc_samples,len(object_info['time_hist'])-2)
                velocities=[(object_info['pos_hist'][-i]-object_info['pos_hist'][-i-1])/(object_info['time_hist'][-i]-object_info['time_hist'][-i-1]) 
                                                            for i in range(1,acc_samp+2)]
                self.blimps[obj_id]['info']['linear']['acc']=np.mean([
                                                                (velocities[-i]-velocities[-i-1])/(object_info['time_hist'][-i]-object_info['time_hist'][-i-1])
                                                                for i in range(1,acc_samp+1)],axis=0)
            

    def get_last_update_time(self, obj_id, update=True):
        if update:
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['time']

    def get_object_pos(self, obj_id, update=True):
        """
        Gets xyz position of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, xyz position of object
        """
        if update:
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['linear']['pos']

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
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['angular']['euler']

    def get_object_vel(self, obj_id, update=True):
        """
        Gets xyz velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, xyz velocity of object
        """
        if update:
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['linear']['vel']

    def get_object_angular_vel(self, obj_id, update=True):
        """
        Gets angular velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, roll, pitch, yaw velocity
        """
        if update:
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['angular']['vel']

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
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['linear']['acc']

    def get_object_angular_acc(self, obj_id, update=True):
        """
        Gets angular velocity of object specified

        @param obj_id: 0<=obj_id<self.num_objects
            index of object to return
        @return: R^3, roll, pitch, yaw acceleration
        """
        if update:
            self.update(obj_ids=[obj_id])
        return self.blimps[obj_id]['info']['angular']['acc']

if __name__=="__main__":
    x=Vicon(['chatter'],'')
    for i in range(10):
        time.sleep(1)
        #print(x.blimps[0]['info']['time_hist'])
        print()
        print(x.get_object_vel(0))
        print()
    quit()
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
        print("nice")


    def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("chatter", TransformStamped, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    listener()
