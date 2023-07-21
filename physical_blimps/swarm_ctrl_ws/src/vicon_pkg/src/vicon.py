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
        self.object_data = dict()
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

            pos=np.array(pos.x,pos.y,pos.z)
            r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            arr=r.as_euler('xyz', degrees=False)
            dictionary[key]['pos_hist'].append(pos)
            dictionary[key]['ang_hist'].append(arr)
            dictionary[key]['time_hist'].append(time)
        return callback



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
if __name__=="__main__":
    import subprocess,os
    os.system("rostopic echo -n 1 /vicon/b2/b2")
    result=subprocess.run(['rostopic','echo','-n','1','/vicon/b2/b2/'], stdout=subprocess.PIPE)
    b=result.stdout
    s=b.decode('utf-8')
    print(s)
    def create_dict(strig_list):
        d=[]

        first_row=strig_list[0]
        first_off=first_row.count('\t')
        key,value=first_row[first_off:].split(':')
        key=key.strip()
        value=value.strip()
        if value:
            return [{key:float(value)}]
        else:
            sub_dict=[]
            i=1
            for i,row in enumerate(strig_list[1:]):
                off=row.count('\t')
                if off<=first_off:
                    break
                sub_dict.append(row)
            sub=create_dict(sub_dict)
            dick=[{key:sub}]
            if i<len(strig_list)-1:
                rest=create_dict(strig_list[i:])
                for d in rest:
                    dick.append(d)
            return dick

    print(create_dict(s.strip().split('\n')))

    quit()
    def callback(data):
        print(data.transform.translation)
        raise Exception('here')


    def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        print("here")
        rospy.init_node('listener', anonymous=True)
        print("heere")

        rospy.Subscriber("chatter", TransformStamped, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    listener()