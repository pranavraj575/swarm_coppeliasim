-- This file contains the defintions for ros2 related messages and topics
TOPIC_PRE = '/swarm/a'
TOPIC_CMD = '/set/cmd_vel'
TOPIC_GLOBAL = '/state/global'
TOPIC_PROXIMITY = '/state/proximity'
TOPIC_SWARM_LIST = '/swarm/list'
TOPIC_RECEIVER = '/state/receiver'
TOPIC_EMITTER = '/set/emitter'
TOPIC_ULTRA = '/state/ultrasonic'

MSG_TYPE_BOOL = 'std_msgs/msg/Bool'
MSG_TYPE_FLOAT_64 = 'std_msgs/msg/Float64'
MSG_TYPE_STRING = 'std_msgs/msg/String'
MSG_TYPE_TWIST = 'geometry_msgs/msg/Twist'
MSG_TYPE_TWISTSTAMPED = 'geometry_msgs/msg/TwistStamped'
MSG_TYPE_VECTOR_3 = 'geometry_msgs/msg/Vector3'