import subprocess
import time, os
from zmqRemoteApi import RemoteAPIClient
import rclpy
import numpy as np

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64

frictionless_wall_path = '/home/rajbhandari/projects/blimp_coppeliasim/scenes/FrictionlessWallClimb.ttt'
empty_path = '/home/rajbhandari/projects/blimp_coppeliasim/scenes/empty.ttt'
narrow_blimp_path = '/home/rajbhandari/projects/blimp_coppeliasim/ros_ctrl_models/blimp_narrow.ttm'


class Experiment:
    def __init__(self,
                 scenePath,
                 sim=None,
                 wakeup=None,
                 sleeptime=1.,
                 ):
        """

        @param scenePath: path to the scene to load
        @param sim: zqm simulator api, if None, than makes its own
        @param wakeup: command to run on initialization (i.e. start coppeliasim)
        @param sleeptime: time to wait after important commands (start/stop/pause simulation)
        """
        self.scenePath = scenePath
        self.sleeptime = sleeptime
        if wakeup is not None:
            subprocess.Popen(wakeup)
            time.sleep(self.sleeptime)
        if sim is not None:
            self.sim = sim
        else:
            client = RemoteAPIClient()
            self.sim = client.getObject('sim')

    ####################################################################################################################
    # init/shutdown functions (needs implementation in subclass)
    ####################################################################################################################

    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        raise NotImplementedError()

    def despawnThings(self):
        """
        to be run at end of each expiriment
        """
        raise NotImplementedError()

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def set_color(self, handle, color, type=None):
        """
        sets object color
        @param handle: object handle
        @param color: [0...255]^3
        @param type: color component, self.sim.colorcomponent_ambient_diffuse is the best other option
        https://www.coppeliarobotics.com/helpFiles/en/apiConstants.htm#colorComponents
        """
        if type is None:
            type = self.sim.colorcomponent_emission
        self.sim.setObjectColor(handle, 0, type, color)

    ####################################################################################################################
    # Expiriment functions (some need implementation in subclass)
    ####################################################################################################################

    def init_exp(self, reset):
        """
        initializes the expiriment
        @param reset: whether to rest the current scene
        """
        if not rclpy.ok():
            rclpy.init()
        if reset:
            while self.sim.simulation_stopped!= self.sim.getSimulationState():
                self.sim.stopSimulation()
                time.sleep(self.sleeptime)
            self.sim.loadScene(os.path.abspath(os.path.expanduser(self.scenePath)))
            #time.sleep(self.sleeptime)
            self.spawnThings()

    def run_exp(self,
                end_time,
                reset=True,
                stop_after=False):
        """
        runs a single expiriment trial
        @param end_time: R+ -> bool, given the time, decide whether to end experiment
            (can obviously use other class variables like object positions to decide)
        @param reset: whether to reset the scene beforehand
        @param stop_after: whether to stop simulation after running experiment
        @return: returns result of self.goal_data
        """
        self.init_exp(reset)
        self.sim.startSimulation()
        while rclpy.ok() and not end_time(self.sim.getSimulationTime()):
            self.step()
        while self.sim.simulation_paused!= self.sim.getSimulationState():
            self.sim.pauseSimulation()
            time.sleep(self.sleeptime)
        #time.sleep(self.sleeptime)
        output = self.goal_data()
        if stop_after:
            while self.sim.simulation_stopped!= self.sim.getSimulationState():
                self.sim.stopSimulation()
                time.sleep(self.sleeptime)
        self.despawnThings()
        return output

    def experiments(self, trials, end_time):
        """
        runs multiple experiments, resetting scene at start of each one
        @param trials: number of experiments to run
        @param end_time: R+ -> bool, given the time, decide whether to end experiment
            (can obviously use other class variables like object positions to decide)
        @return: returns list of results of self.goal_data for each trial
        """
        results = []
        for trial in range(trials):
            data = self.run_exp(
                end_time=end_time,
                reset=True,
                stop_after=False
            )
            results.append(data)
        return results

    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)
        """
        raise NotImplementedError()

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        can be any type
        """
        raise NotImplementedError()


class BlimpExperiment(Experiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 sim=None,
                 wakeup=None,
                 sleeptime=1.,
                 ):
        """

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param sim: simulator, if already defined
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        """
        super().__init__(
            scenePath=scenePath,
            sim=sim,
            wakeup=wakeup,
            sleeptime=sleeptime,
        )
        self.num_agents = num_agents
        self.start_zone = start_zone
        self.modelPath = blimpPath
        self.agentData = dict()

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnBlimp(self, modelPath, pos_rng, tries=100):
        """
        spawns a model in a certian area
        @param modelPath: path to .ttm model to spawn
        @param pos_rng: (R U R^2)^3, range to spawn into
            (if single value, this is the value for the respective coordinate)
            (if tuple, than uniformly chooses from (low,high))
        @param tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        @return: handle of model spawned
        """
        agentHandle = self.sim.loadModel(os.path.abspath(os.path.expanduser(modelPath)))
        for _ in range(tries):
            Pos = []
            for k in range(3):
                try:
                    Pos.append(np.random.uniform(pos_rng[k][0], pos_rng[k][1]))
                except:
                    Pos.append(pos_rng[k])
            self.sim.setObjectPosition(agentHandle, -1, Pos)
            collisionResult, collidingObjectHandles = self.collision_check(agentHandle)
            if not collisionResult:
                break
        return agentHandle

    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        TOPIC_PRE = '/swarm/a'
        TOPIC_CMD = '/set/cmd_vel'
        TOPIC_GLOBAL = '/state/global'
        TOPIC_ULTRA = '/state/ultrasonic'

        if not rclpy.ok():
            rclpy.init()
        self.agentData = dict()
        for i in range(self.num_agents):
            this_agent = dict()
            this_agent['agentHandle'] = self.spawnBlimp(self.modelPath, self.start_zone(i), 100)
            this_agent['agent_id'] = i

            topicGlobal = TOPIC_PRE + str(i) + TOPIC_GLOBAL
            topicUltra = TOPIC_PRE + str(i) + TOPIC_ULTRA

            topicCmdVel = TOPIC_PRE + str(i) + TOPIC_CMD

            nodeCtrl = rclpy.create_node('lta_' + str(i) + '_publisher')
            vec_publisher = nodeCtrl.create_publisher(Twist, topicCmdVel, 10)

            this_agent['nodeCtrl'] = nodeCtrl
            this_agent['topicCmdVel'] = topicCmdVel
            this_agent['vec_publisher'] = vec_publisher

            nodeState = rclpy.create_node('lta_' + str(i) + '_reciever')
            callback = self.create_callback_twist(this_agent, 'state')
            state_subscriber = nodeState.create_subscription(TwistStamped,
                                                             topicGlobal,
                                                             callback,
                                                             10)
            this_agent['nodeState'] = nodeState
            this_agent['topicGlobal'] = topicGlobal
            this_agent['state_subscriber'] = state_subscriber

            ultracallback = self.create_callback_float(this_agent, 'state')
            nodeUltra = rclpy.create_node('lta_' + str(i) + '_ultra')  # for recieving ultrasound
            ultra_subscriber = nodeUltra.create_subscription(Float64,
                                                             topicUltra,
                                                             ultracallback,
                                                             10)
            this_agent['nodeUltra'] = nodeUltra
            this_agent['topicUltra'] = topicUltra
            this_agent['ultra_subscriber'] = ultra_subscriber

            self.agentData[i] = this_agent

    def despawnThings(self):
        """
        to be run at end of each expiriment
        """
        for agent_id in self.agentData:
            for node_key in ('nodeUltra', 'nodeState', 'nodeCtrl'):
                self.agentData[agent_id][node_key].destroy_node()

    ####################################################################################################################
    # ROS functions
    ####################################################################################################################
    def spin(self, agent_ids=None):
        """
        spins the sensor nodes for given list of agents
        @param agent_ids: agents to 'sense', if None does all agents
        """
        if agent_ids is None:
            agent_ids = self.agentData.keys()
        for agent_id in agent_ids:
            rclpy.spin_once(self.agentData[agent_id]['nodeState'], timeout_sec=.01)
            rclpy.spin_once(self.agentData[agent_id]['nodeUltra'], timeout_sec=.01)

    def create_callback_twist(self, dictionary, key, state_keys=('x', 'y', 'z', 'w')):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state
        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_keys: keys to put x,y,z,w values
        @return: returns callback function to be used in ROS subscriber
        """
        if key not in dictionary:
            dictionary[key] = dict()
        # default values of 0
        dictionary[key].update({k: 0. for k in state_keys})

        def callback(msg):
            dictionary[key][state_keys[0]] = msg.twist.linear.x
            dictionary[key][state_keys[1]] = msg.twist.linear.y
            dictionary[key][state_keys[2]] = msg.twist.linear.z
            dictionary[key][state_keys[3]] = msg.twist.angular.z

        return callback

    def create_callback_float(self, dictionary, key, state_key='ultra'):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state
        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_key: key to put float values
        @return: returns callback function to be used in ROS subscriber
        """
        if key not in dictionary:
            dictionary[key] = dict()
        # default value of 0
        dictionary[key].update({state_key: 0.})

        def callback(msg):
            dictionary[key][state_key] = msg.data

        return callback

    ####################################################################################################################
    # Agent functions
    ####################################################################################################################
    def collision_check(self, agentHandle, handle2=None):
        """
        returns whether agent is colliding with some object
        @param agentHandle: handle to check collision
        @param handle2: handle to check collision
            (if None, then checks all objects)
        @return: (whether collisiion is detected, list of object handle collisions)
        """
        collidingObjectHandles = []
        if handle2 is None:
            handle2 = self.sim.handle_all
        result = self.sim.checkCollision(agentHandle, handle2)
        if type(result) is tuple:
            collisionResult = result[0]
            collidingObjectHandles = result[1]
        else:
            collisionResult = result
        collisionResult = bool(collisionResult)
        return collisionResult, collidingObjectHandles

    def move_agent(self, agent_id, vec):
        """
        publishes a vector to an agent
            (currently publishes a velocity goal, and LUA controls in blimpNew.lua takes care of rest)
        @param agent_id: agent id to publish to
        @param vec: vector to publish

        @note: currently using just the linear part of twist message,
            can use orientation for other stuff if we update blimpNew.lua
        """
        if agent_id not in self.agentData:
            raise Exception("attempted to move agent that does not exist: id of " + str(agent_id))
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])
        msgTwist.linear.y = float(vec[1])
        msgTwist.linear.z = float(vec[2])

        self.agentData[agent_id]['vec_publisher'].publish(msgTwist)

    def get_state(self, agent_id, spin=True):
        """
        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: dictionary
        @return: state of agent
        """
        if spin:
            self.spin([agent_id])
        return self.agentData[agent_id]['state']

    def get_position(self, agent_id, use_ultra=False, spin=True):
        """
        returns position of agent
        @param agent_id: agent id
        @param use_ultra: whether to use ultrasound sensor as opposed to state['z']
        @param spin: whether to update agent before getting state
        @rtype: R^3 numpy array
        @return: position of agent
        """
        s = self.get_state(agent_id, spin=spin)
        return np.array((s['x'], s['y'], s['ultra' if use_ultra else 'z']))

    def _gen_get_neighbors(self, agent_id, is_neigh, spin=False):
        """
        gets neighbors of an agent, general implementation
        @param agent_id: agent id
        @param is_neigh: agent_id1 x agent_id2 -> bool; returns if agent 2 is a neighbor of agent 1
        @param spin: whether to update all agents before checking neighbors
        @return: list of agent ids; neighbors of agent
        """
        if spin:
            self.spin()
        return [neigh_id for neigh_id in self.agentData if
                (neigh_id != agent_id and is_neigh(agent_id, neigh_id))]

    def get_neighbors_range(self, agent_id, rng, spin=False):
        """
        gets neighbors of an agent, range implementation
        @param agent_id: agent id
        @param rng: radius around agent to register as a neighbor (in 3d)
        @param spin: whether to update all agents before checking neighbors
        @return: list of agent ids; neighbors of agent
        """
        return self._gen_get_neighbors(agent_id, lambda id1, id2: self.within_range(id1, id2, rng, spin=False), spin)

    def get_neighbors_3d_l_k_ant(self, agent_id, rng, k=8, l=2, spin=True):
        """
        gets count of neighbors in each l,k-ant
            i.e. l=2, k=1 is equivalent to 'north hemisphere, south hemisphere'
            l=2 k=4 is equivalent to octants of a sphere
        @param agent_id: agent id
        @param rng: range for which agents count as neighbors
        @param k: divisions of theta to consider when using spherical coordinates
        @param l: divisions of phi to consider when using spherical coordinates
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^(l x k) array
        @return: return[i][j] specifies number of neighbors in the range phi=pi*[i,i+1)/l and theta=2*pi*[j,j+1)/k
        """
        neighbors = self.get_neighbors_range(agent_id, rng, spin=spin)
        output = [[0 for _ in range(k)] for _ in range(l)]
        pos = self.get_position(agent_id, use_ultra=False, spin=False)
        for neigh_id in neighbors:
            neigh_pos = self.get_position(neigh_id, use_ultra=False, spin=False)
            vec = neigh_pos - pos
            l_tant, k_tant = self._get_l_k_tant(vec=vec, l=l, k=k)
            output[l_tant][k_tant] += 1
        return output

    def get_neighbors_2d_k_ant(self, agent_id, rng, k=8, spin=True):
        """
        gets count of neighbors in each k-ant
            i.e. k=4 is equivalent to quadrants of the xy plane
        @param agent_id: agent id
        @param rng: range for which agents count as neighbors
        @param k: divisions of theta to consider when using spherical coordinates
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^k array
        @return: return[i] specifies number of neighbors in the approximate direction ()

        @note: when getting neighbors with range, z direction is considered,
        """
        return self.get_neighbors_3d_l_k_ant(agent_id=agent_id,
                                             rng=rng,
                                             k=k,
                                             l=1,
                                             spin=spin)[0]

    ####################################################################################################################
    # utility functions
    ####################################################################################################################

    def within_range(self, id1, id2, rng, spin=False):
        """
        checks whether id1 agent is within 'range' of id2 agent
        @param id1: agent 1 id
        @param id2: agent 2 id
        @param rng: R, range to check
        @param spin: whether to update each object before getting position
        @return: boolean, whether the two agents are close enough
        """
        pos1 = self.get_position(id1, use_ultra=False, spin=spin)
        pos2 = self.get_position(id2, use_ultra=False, spin=spin)
        return np.linalg.norm(pos1 - pos2) <= rng

    def _get_l_k_tant(self, vec, l, k):
        """
        gets l,k-tant of vec
            i.e. l=2, k=1 is equivalent to 'north hemisphere, south hemisphere'
            l=2 k=4 is equivalent to octants of a sphere
        @param vec: vector to check
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
        @return: l-tant, k-tant;
            i.e. return of i,j means in spherical coords, vec is in range phi=pi*[i,i+1)/l and theta=2*pi*[j,j+1)/k
        """
        theta = np.arctan2(vec[1], vec[0]) % (2 * np.pi)
        phi = np.pi / 2 - np.arctan2(vec[2], np.linalg.norm(vec[:2]))

        l_tant = int(l * phi / (np.pi))
        # is in range [0,l], needs to be clipped if phi=pi

        k_tant = int(k * theta / (2 * np.pi))
        # will always be in range [0,k)
        return min(l - 1, l_tant), k_tant


class blimpTest(BlimpExperiment):
    def __init__(self, num_agents, start_zone, scene_path=empty_path, blimp_path=narrow_blimp_path):
        super().__init__(num_agents, start_zone, scene_path, blimp_path)

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        just moves towards next agent's position
        """
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, use_ultra=False, spin=True)
            next_pos = self.get_position((agent_id + 1) % self.num_agents, use_ultra=False, spin=True)
            # goal = np.array((1, 0, agent_id * .5 + 1))
            goal = next_pos
            vec = goal - pos
            #vec = np.zeros(3)
            self.move_agent(agent_id, vec * .1)

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        returns z position right now
        """
        s = []
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, use_ultra=False)
            s.append(pos[2])
        return s



if __name__ == "__main__":
    bb = blimpTest(10, lambda i: ((-5, 5), (-5, 5), (1, 5)))
    bb.run_exp(end_time=lambda t: False)
