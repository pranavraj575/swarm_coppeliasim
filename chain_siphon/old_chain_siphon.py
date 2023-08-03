#!/usr/bin/env python3

import rclpy
import os
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import time
from std_msgs.msg import Float64
import sys
import copy
from collections import defaultdict

import numpy as np

OUT = ''
PI = 3.14159
SIMID = 23000

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

msgfile = os.path.join(DIR, 'lua', 'rosMsg.lua')
TOPIC_NAMES = dict()
with  open(msgfile) as f:
    r = [t.split('=') for t in f.read().strip().split('\n') if '=' in t]
    for key, item in r:
        TOPIC_NAMES[key.strip()] = item.strip().replace("'", '')

TOPIC_PRE = TOPIC_NAMES['TOPIC_PRE_BLIMP']
TOPIC_CMD = TOPIC_NAMES['TOPIC_CMD']
TOPIC_GLOBAL = TOPIC_NAMES['TOPIC_GLOBAL']
TOPIC_ULTRA = TOPIC_NAMES['TOPIC_ULTRA']

SWARM_DATA = defaultdict(lambda: np.array([0.]*5))


class Chains:
    def __init__(self,
                 agent_id,
                 num_agents,
                 goal_value=lambda pos: 0.,
                 goal_field=None,
                 swarm_data=SWARM_DATA,
                 debug=False):
        """
        :param agent_id is the number that the agent publishes to
        :param num_agents is the number of agents in the environments
        :param goal_value R^3->R objective function to be minimized
        :param goal_field R^3->R^3, gradient of goal_value, points towards lowest potential (e.g. use radial function if goal is to a point)
        """
        self.agent_id = agent_id
        self.num_agents = num_agents
        self.swarm_data = swarm_data

        topicGlobal = TOPIC_PRE + str(SIMID) + '_' + str(self.agent_id) + TOPIC_GLOBAL
        topicUltra = TOPIC_PRE + str(SIMID) + '_' + str(self.agent_id) + TOPIC_ULTRA

        topicCmdVel = TOPIC_PRE + str(SIMID) + '_' + str(self.agent_id) + TOPIC_CMD

        self.nodeCtrl = rclpy.create_node(
            'lta' + str(SIMID) + '_' + str(self.agent_id) + '_chain_publisher')  # for publishing

        self.vec_publisher = self.nodeCtrl.create_publisher(Twist, topicCmdVel, 10)
        # for actuation, give it a position, it goes for it, maybe can replace with velocity?

        # for getting goal position, can replace with vector field function
        self.set_goal_value(goal_value=goal_value, goal_field=goal_field)

        self.state = {
            'x': 0,
            'y': 0,
            'z': 0,
            'w': 0,
            'ultra': 0,
        }

        self.nodeState = rclpy.create_node(
            'lta' + str(SIMID) + '_' + str(self.agent_id) + '_chain_reciever')  # for recieving
        self.state_subscriber = self.nodeState.create_subscription(TwistStamped, topicGlobal, self.callbackUpdateState,
                                                                   10)
        self.nodeUltra = rclpy.create_node(
            'lta' + str(SIMID) + '_' + str(self.agent_id) + '_chain_ultra')  # for recieving ultrasound
        self.ultra_subscriber = self.nodeUltra.create_subscription(Float64,
                                                                   topicUltra,
                                                                   self.callbackUpdateUltra,
                                                                   10)
        # for getting state
        self.debug = debug
        if self.debug:
            print('state from', topicGlobal)

        self.spin()

    def break_chain(self):
        """
        the opposite of the init method
        """
        # destroy them
        self.nodeCtrl.destroy_node()
        self.nodeState.destroy_node()
        self.nodeUltra.destroy_node()

    def set_goal_value(self, goal_value, goal_field=None):
        """
        :param goal_value: R^3->R objective function to be minimized
        :param goal_field R^3->R^3, gradient of goal_value, points towards lowest potential (e.g. use radial function if goal is to a point)
        """
        self.goal_value = goal_value
        if goal_field == None:
            d = .01
            goal_field = lambda p: -np.array([
                (self.goal_value(p + np.array((d, 0, 0))) - self.goal_value(p + np.array((-d, 0, 0))))/(2*d),
                (self.goal_value(p + np.array((0, d, 0))) - self.goal_value(p + np.array((0, -d, 0))))/(2*d),
                (self.goal_value(p + np.array((0, 0, d))) - self.goal_value(p + np.array((0, 0, -d))))/(2*d)
            ])
        self.goal_field = goal_field

    def callbackUpdateState(self, msg):
        # print('here')
        self.state['x'] = msg.twist.linear.x
        self.state['y'] = msg.twist.linear.y
        self.state['z'] = msg.twist.linear.z
        self.state['w'] = msg.twist.angular.z
        # print(state)

    def callbackUpdateUltra(self, msg):
        # print('here')
        self.state['ultra'] = msg.data
        # print(state)

    def dataCallback(self, data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        print(self.agent_id, 'READING')
        self.data = copy.deepcopy(data)
        print(self.data.data)

    def data_publish(self, agent_data):
        self.swarm_data[self.agent_id] = np.array(agent_data)

    def get_state(self, spin=True):
        if spin:
            self.spin()
        return self.state

    def get_position(self, use_ultra=False, spin=True):
        s = self.get_state(spin=spin)
        return np.array((s['x'], s['y'], s['ultra' if use_ultra else 'z']))

    def move(self, vec):
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])
        msgTwist.linear.y = float(vec[1])
        msgTwist.linear.z = float(vec[2])

        self.vec_publisher.publish(msgTwist)

    def spin(self):
        rclpy.spin_once(self.nodeState, timeout_sec=.01)
        rclpy.spin_once(self.nodeUltra, timeout_sec=.01)

    def hover(self):
        # for testing
        self.spin()
        self.move((0, 0, .0))

    def safe_norm(self, vec, d=.00001):
        size = np.linalg.norm(vec)
        if size <= 0:
            size = d
        return vec/size

    def point_obs_force(self, position, point_obstacles, obs_range=1.5, obs_rep=10):
        out = np.zeros(3)
        for pt in point_obstacles:
            v_obs = pt - position

            dist = np.linalg.norm(v_obs)
            if dist == 0:
                dist = 0.00001

            if dist < obs_range:
                v_obs = self.safe_norm(v_obs)
                v_obs *= (-1*(obs_rep*1*1)/(dist**2))
            else:
                v_obs = np.zeros(3)

            out += v_obs
        return out

    def siphon(self,
               # (goal, obstacle, viscosity, min, bounding)
               # weight=(1.0, 1.0, 0.05, 20, 50),
               # weight=(1.0, 1.0, 0.05, 20, 1),
               weight=(.1, .1, .001, .4, .1),
               point_obstacles=(),
               obs_vector=lambda pos: np.array((0, 0, 0)),
               max_speed=0.75,
               obs_range=1.5,
               obs_rep=10,
               etta=0.4,
               neighbor_range=2.25,  # 1.75,
               max_ljp=500,
               goal_area=(-5, 0, 1),
               # workspace=((-5, 6), (-1.5, 4), (-float('inf'), float('inf'))),
               workspace=((-5, 6), (-1.5, 4), (1, 1.25)),
               # goal_test=lambda p: (p[0] < -1.5) or (p[0] < 0 and p[1] > 1.5),
               goal_test=lambda p: False,
               use_ultra=True,
               ):
        """

        :param weight: weights of each vector (goal, obstacle, viscosity, min, bounding)
        :param point_obstacles: point obstacles to avoid
        :param obs_vector: R^3->R^3 vector to force blimps from obstacle (0,0,0 if no force)
        :param initial_speed: initial speed vector
        :param max_speed: max speed vector
        :param obs_range: range that obstacle avoidance field extends to
        :param obs_rep: 'repulsion' of the obstacle (vector is obs_rep/dist^2)
        :param etta: etta parameter for LJP
        :param max_ljp: maximum value of total LJP vector (so it cannot approach infinity)
        :param neighbor_range: range to identify neighbors
        :param workspace: area to work in
        :param goal_test: true if the queue value is 1
        :param use_ultra: whether to use ultrasonic for z value
        """
        height = None
        # node=rclpy.create_node('agent_'+str(self.agent_id)+'_rate')
        # thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        # thread.start()
        # rate = node.create_rate(10)
        # while rclpy.ok():# not rclpy.is_shutdown()
        self.spin()
        pos = self.get_position(use_ultra=use_ultra, spin=False)

        real_pos = self.get_position(use_ultra=False, spin=False)
        # for real x,y,z

        #####################################################################################################
        # Goal (normalized)

        v_goal = self.safe_norm(self.goal_field(real_pos))

        #####################################################################################################
        # Obstacles
        v_obs_tot = np.zeros(3)
        v_obs_tot += self.point_obs_force(real_pos, point_obstacles, obs_range, obs_rep)
        v_obs_tot += obs_vector(real_pos)

        #####################################################################################################
        # LJP

        neigh_min_queue_pos = self.num_agents
        min_dist = neighbor_range
        min_dist_element = None
        v_visc = np.zeros(3)
        neighbors = []
        for key in self.swarm_data:
            item = self.swarm_data[key]
            x = item[0]
            if x != self.agent_id:
                # TODO: mess with agent id/ whatever
                dPos = item[1:4] - real_pos
                dist = np.linalg.norm(dPos)
                if dist <= 0:
                    dist = .00001
                if dist < neighbor_range:
                    neighbors.append(item)  # (x)
                    # among neighbors, find the minimum queue position
                    if item[4] < neigh_min_queue_pos:
                        neigh_min_queue_pos = item[4]
                        min_dist = dist
                        min_dist_element = item
                    # among elements with queue position "minimum", find minimum distance
                    if item[4] == neigh_min_queue_pos and dist < min_dist:
                        min_dist = dist
                        min_dist_element = item

        #####################################################################################################
        # Siphon

        queue_value = self.num_agents
        v_min = np.zeros(3)

        new_v_min = np.zeros(3)
        best_leader = -1
        obj_value = self.goal_value(real_pos)

        best_value = None

        static_goal = goal_area - real_pos

        if goal_test(real_pos):
            queue_value = 1

        if neighbors:
            min_size = self.num_agents + 1
            for item in neighbors:
                dPos = item[1:4] - real_pos

                dist = np.linalg.norm(dPos)
                if dist == 0:
                    dist = 0.00001

                dPos = self.safe_norm(dPos)

                ljp = dPos*(24*etta*(-26/((0.6*(dist + 0.75))**14) + 7/((0.6*(dist + 0.75))**8)))

                if item[4] < queue_value:
                    queue_value = item[4] + 1

                if queue_value > self.num_agents:
                    queue_value = self.num_agents

                # if item[4] == minimum and dist == min_dist and minimum < queue_value:
                if item is min_dist_element and neigh_min_queue_pos < queue_value:
                    v_min = dPos

                v_visc += ljp

                ## NEW doesnt this work?
                visited = {self.agent_id}
                next = item[0]
                mm = float('inf')
                siz = self.num_agents + 2
                while next >= 0 and next not in visited:
                    fol = self.swarm_data[next]
                    if fol[6] < mm:
                        mm = fol[6]
                        siz = len(visited)
                    visited.add(fol[0])
                    next = fol[5]

                if mm < obj_value:
                    # if other agent is 'better' or following somone better than this agent
                    if best_value is None or mm < best_value or (mm == best_value and siz < min_size):
                        # follow the best possible agent
                        best_value = mm
                        best_leader = item[0]
                        new_v_min = dPos
                        min_size = siz
        if np.linalg.norm(v_visc) > max_ljp:
            v_visc = self.safe_norm(v_visc)*max_ljp
        #####################################################################################################
        # Workspace
        v_bound = np.zeros(3)
        # USE ULTRASOUND POS HERE, matches the 'climbing' behavior
        for k in range(3):
            temp = np.zeros(3)
            if pos[k] < workspace[k][0]:
                temp[k] += workspace[k][0] - pos[k]  # 1.
            if pos[k] > workspace[k][1]:
                temp[k] += workspace[k][1] - pos[k]  # -1.
            v_bound += temp

        #####################################################################################################
        # Full

        v_full = v_goal*weight[0] + \
                 v_obs_tot*weight[1] + \
                 v_visc*weight[2] + \
                 new_v_min*weight[3] + \
                 v_bound*weight[4]
        if height:
            v_full[2] = (height - pos[2])*.1
            # USE ULTRASOUND POS HERE
        if np.linalg.norm(v_full) > max_speed:
            v_full = self.safe_norm(v_full)*max_speed
        p_full = v_full + real_pos
        disp_weights = list(weight) + [1.]

        vecs = (v_goal, v_obs_tot, v_visc, new_v_min, v_bound, v_full)
        if self.debug:
            global OUT
            print(self.agent_id, [[round(value, 2) for value in v] for v in vecs])
            # output = [round(np.linalg.norm(vecs[k] * disp_weights[k]), 2) for k in range(len(vecs))]
            # OUT += str(self.agent_id) + ' ' + str(output) + '\n'

        self.move(v_full)
        self.data_publish((self.agent_id,
                           real_pos[0],
                           real_pos[1],
                           real_pos[2],
                           queue_value,
                           best_leader,
                           obj_value,
                           len(neighbors)
                           ))
        # rate.sleep()
        # self.spin()


from zmqRemoteApi import RemoteAPIClient
import random


def checkForCollisionWrapper(agentHandle, sim):
    collisionResult = None
    collidingObjectHandles = None
    result = sim.checkCollision(agentHandle, sim.handle_all)
    if type(result) is tuple:
        collisionResult = result[0]
        collidingObjectHandles = result[1]
    else:
        collisionResult = result
    collisionResult = bool(collisionResult)
    return collisionResult, collidingObjectHandles


if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    startZone = {'xmin': 1, 'xmax': 5,
                 'ymin': -1.5, 'ymax': 4,
                 'zmin': 1, 'zmax': 1,
                 }

    DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
    MODELDIR = DIR + '/ros_ctrl_models/blimp_narrow.ttm'
    SCENEDIR = DIR + '/scenes/empty.ttt'
    narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
    modelToLoad = narrowModelPath

    sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
    sim.stopSimulation()
    time.sleep(1)

    sim.loadScene(sceneNamePath)
    time.sleep(1)

    agentHandles = []
    chains = []
    rclpy.init()
    num_agents = 10


    def make_goal_radial(goal):
        return lambda pos: np.linalg.norm(pos - goal)**2


    for i in range(num_agents):
        agentHandle = sim.loadModel(modelToLoad)
        for _ in range(100):  # give up at some point
            agentHandles.append(agentHandle)  # Save loaded models' handles for analysis later
            xPos = round(random.uniform(startZone["xmin"], startZone["xmax"]), 2)
            yPos = round(random.uniform(startZone["ymin"], startZone["ymax"]), 2)
            zPos = round(random.uniform(startZone["zmin"], startZone["zmax"]), 2)
            sim.setObjectPosition(agentHandle, -1, [xPos, yPos, zPos])
            collisionResult, collidingObjectHandles = checkForCollisionWrapper(agentHandle, sim)
            if not collisionResult:
                break
        R = 2.5
        pp = Chains(i,
                    num_agents,
                    # lambda pos:pos[0]
                    # lambda pos:np.cos(pos[0])-(pos[1]-1)**2-((pos[2]-1)**2/100 if pos[2]>2 or pos[2]<.5 else 0)
                    # make_goal_radial(np.array((0, 0, 10)))
                    make_goal_radial(np.array(
                        (R*np.sin(2*i*PI/num_agents), 0., R*np.cos(2*i*PI/num_agents) + R + .1)))
                    )
        chains.append(pp)

    sim.startSimulation()
    time.sleep(1)

    while False:  # rclpy.ok():
        s = time.time()
        for i in range(len(chains)):
            chains[i].siphon()
        # for thingy in SWARM_DATA:
        # print(thingy, [round(v,2) for v in SWARM_DATA[thingy]])
        OUT = OUT + 'time: ' + str(round(time.time() - s, 3)) + '\n'
        print(OUT)
        OUT = ''
        print()

    eye = 0
    while True:
        eye += 1
        for j in range(len(chains)):
            pp = chains[j]
            pos = pp.get_position(use_ultra=False)
            size = .1
            vec = pp.goal_field(pos)*.1
            if np.linalg.norm(vec) > size:
                vec = size*vec/np.linalg.norm(vec)
            pp.move(vec)
            if j == 0:
                # print([round(p,4) for p in (pos-old_poses[j])/(50/1000)],' '*10,end='\r')
                # print([round(v,2) for v in vec], ' ' * 20, end='\r')
                # print(vec)
                i = 0
                # print(chains[0].state['z'],chains[0].state['ultra'])
