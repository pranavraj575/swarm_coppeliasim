import subprocess, psutil
import time, os, sys
import rclpy
import numpy as np

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64

from CONFIG import *

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

msgfile = os.path.join(DIR, 'lua', 'rosMsg.lua')
TOPIC_NAMES = dict()
with  open(msgfile) as f:
    r = [t.split('=') for t in f.read().strip().split('\n') if '=' in t]
    for key, item in r:
        TOPIC_NAMES[key.strip()] = item.strip().strip("'")

frictionless_wall_path = os.path.join(DIR, 'scenes', 'FrictionlessWallClimb.ttt')
wall_climb_path = os.path.join(DIR, 'scenes', 'WallClimb.ttt')
wall_climb_undetect_path = os.path.join(DIR, 'scenes', 'WallClimbUndetectable.ttt')
caged_wall_climb_path = os.path.join(DIR, 'scenes', 'wall_climb_caged.ttt')
cage_arena_path = os.path.join(DIR, 'scenes', 'cage_arena_better.ttt')
anki_arena_path = os.path.join(DIR, 'scenes', 'ankiArena.ttt')
anki_large_arena_path = os.path.join(DIR, 'scenes', 'largeAnkiArena.ttt')
empty_path = os.path.join(DIR, 'scenes', 'empty.ttt')

narrow_blimp_path = os.path.join(DIR, 'ros_ctrl_models', 'blimp_narrow.ttm')
anki_path = os.path.join(DIR, 'ros_ctrl_models', 'anki_visionless.ttm')
quad_path = os.path.join(DIR, 'ros_ctrl_models', 'quad_copter.ttm')


class Experiment:
    def __init__(self,
                 scenePath,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=.01,
                 random_seed=None,
                 ):
        """
        General class that intializes and runs a repeatable experiment, returning results

        @param scenePath: path to the scene to load
        @param sim: zqm simulator api, if None, than makes its own
        @param simId: simulator id, used to pass messages to correct topics
        @param wakeup: list of commands to run on initialization (i.e. start coppeliasim)
        @param sleeptime: time to wait after important commands (start/stop/pause simulation)
        @param random_seed: seed to start np.random at
            if None, uses simId and current time to create a hopefully unique one
        """
        if random_seed is None:
            t = str(time.time())
            t = t[t.index('.') + 1:]
            np.random.seed(int(t + str(simId))%2**32)

        self.scenePath = scenePath
        self.sleeptime = sleeptime
        self.procs = []
        self.simId = simId

        if wakeup is not None:
            for cmd in wakeup:
                self.procs.append(subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True).pid)
            time.sleep(self.sleeptime)
        if sim is not None:
            self.sim = sim
        else:
            from zmqRemoteApi import RemoteAPIClient
            # NECESSARY to make this work with multiple simulators
            client = RemoteAPIClient(port=simId)
            self.sim = client.getObject('sim')
        self.problem_models = dict()

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

    def kill(self):
        """
        destroys all subprocesses
        """
        for proc_pid in self.procs:
            process = psutil.Process(proc_pid)
            for proc in process.children(recursive=True):
                proc.kill()
            process.kill()
        self.procs = []

    def close_zmq(self):
        """
        closes the zmq socket
        """
        del self.sim

    ####################################################################################################################
    # Expiriment functions (some need implementation in subclass)
    ####################################################################################################################
    def init_exp(self, reset):
        """
        initializes the expiriment

        @param reset: whether to rest the current scene
        """
        self.clear_problem_modles()
        while True:
            try:
                if not rclpy.ok():
                    rclpy.init()
                break
            except:
                time.sleep(self.sleeptime)
        if reset:
            while self.sim.simulation_stopped != self.sim.getSimulationState():
                self.sim.stopSimulation()
                time.sleep(self.sleeptime)
            self.sim.loadScene(os.path.abspath(os.path.expanduser(self.scenePath)))
            # time.sleep(self.sleeptime)
            self.spawnThings()

    def run_exp(self,
                reset=True,
                stop_after=False):
        """
        runs a single expiriment trial

        @param reset: whether to reset the scene beforehand
        @param stop_after: whether to stop simulation after running experiment
        @return: returns result of self.goal_data
        """
        self.init_exp(reset)
        self.sim.startSimulation()
        while rclpy.ok():
            done = self.step()
            if done:
                break
        while self.sim.simulation_paused != self.sim.getSimulationState():
            self.sim.pauseSimulation()
            time.sleep(self.sleeptime)
        # time.sleep(self.sleeptime)
        output = self.goal_data()
        if stop_after:
            while self.sim.simulation_stopped != self.sim.getSimulationState():
                self.sim.stopSimulation()
                time.sleep(self.sleeptime)
        self.despawnThings()
        return output

    def experiments(self, trials):
        """
        runs multiple experiments, resetting scene at start of each one

        @param trials: number of experiments to run
        @return: returns list of results of self.goal_data for each trial
        """
        results = []
        for trial in range(trials):
            data = self.run_exp(
                reset=True,
                stop_after=False
            )
            if data is None:
                return None
            results.append(data)
        return results

    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        raise NotImplementedError()

    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: can be anything, None signals error
        """
        raise NotImplementedError()

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def moveObject(self, handle, pos_rng, orient_rng=None):
        """

        @param handle: object handle
        @param pos_rng: ()->(R U R^2)^3, range to spawn into
            (if single value, this is the value for the respective coordinate)
            (if tuple, than uniformly chooses from (low,high))
        @param orient_rng: ()->(R U R^2)^3, range of orientations to spawn into,
            None if default model orientation
            roll, pitch, yaw
        """
        Pos = []
        rng = pos_rng()
        for k in range(3):
            try:
                Pos.append(np.random.uniform(rng[k][0], rng[k][1]))
            except:
                Pos.append(float(rng[k]))
        self.sim.setObjectPosition(handle, -1, Pos)
        if orient_rng is not None:
            Orient = []
            rng = orient_rng()
            for k in range(3):
                try:
                    Orient.append(np.random.uniform(rng[k][0], rng[k][1]))
                except:
                    Orient.append(float(rng[k]))
            self.sim.setObjectOrientation(handle, -1, Orient)

    def spawnModel(self, modelPath, pos_rng, spawn_tries, orient_rng=None):
        """
        spawns a model in a certian area

        @param modelPath: path to .ttm model to spawn
        @param pos_rng: ()->(R U R^2)^3, range to spawn into
            (if single value, this is the value for the respective coordinate)
            (if tuple, than uniformly chooses from (low,high))
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        @param orient_rng: ()->(R U R^2)^3, range of orientations to spawn into,
            None if default model orientation
            roll, pitch, yaw
        @return: handle of model spawned
        """
        agentHandle = self.sim.loadModel(os.path.abspath(os.path.expanduser(modelPath)))
        for _ in range(spawn_tries):
            self.moveObject(handle=agentHandle, pos_rng=pos_rng, orient_rng=orient_rng)
            collisionResult, collidingObjectHandles = self.collision_check(agentHandle)
            if not collisionResult:
                break
        return agentHandle

    def collision_check(self, agentHandle, handle2=None, static=True):
        """
        returns whether agent is colliding with some object

        @param agentHandle: handle to check collision
        @param handle2: handle to check collision
            (if None, then checks all objects)
        @param static: whether no models moved from last time
            if true, speeds up bounding box check
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
        if self.problem_models:
            for handle in self.problem_models:
                box = self.problem_bounding_box(handle, static=static)
                pt = self.get_object_pos(agentHandle)
                if self.within_box(pt, box):
                    collisionResult = True
                    collidingObjectHandles.append(handle)
        return collisionResult, collidingObjectHandles

    def get_object_pos(self, handle):
        """
        returns position of object
        @param handle: handle of object
        @return: R^3 position
        """
        return np.array(self.sim.getObjectPosition(handle, self.sim.handle_world))

    def problem_bounding_box(self, handle, static):
        """
        returns bounding box of problem object
            speedup if this has been accessed before
        @param handle: handle of object
        @param static: if True, object has not moved
        @return: (R x R)^3 bounding box
        """
        if static:
            if self.problem_models[handle] is not None:
                return self.problem_models[handle]
        box = self.bounding_box(handle)
        self.problem_models[handle] = box
        return box

    def bounding_box(self, handle):
        """
        returns bounding box of object
        @param handle: handle of object
        @return: (R x R)^3 bounding box
        """
        box = np.zeros((3, 2)) + self.get_object_pos(handle).reshape((3, 1))
        box[0][0] += self.sim.getObjectFloatParameter(handle, self.sim.objfloatparam_objbbox_min_x)[1]
        box[0][1] += self.sim.getObjectFloatParameter(handle, self.sim.objfloatparam_objbbox_max_x)[1]

        box[1][0] += self.sim.getObjectFloatParameter(handle, self.sim.objfloatparam_objbbox_min_y)[1]
        box[1][1] += self.sim.getObjectFloatParameter(handle, self.sim.objfloatparam_objbbox_max_y)[1]

        box[2][0] += self.sim.getObjectFloatParameter(handle, self.sim.objfloatparam_objbbox_min_z)[1]
        box[2][1] += self.sim.getObjectFloatParameter(handle, self.sim.objfloatparam_objbbox_max_z)[1]
        return box

    @staticmethod
    def box_overlap(box0, box1):
        """
        returns if bounding boxes overlap
        @param box0: first box
        @param box1: second box
        """
        return all(
            any(box0[dim][0] < box1[dim][i] and box1[dim][i] < box0[dim][1] for i in range(2))
            # overlaps if either the minimum coord of box 1 is between the bounds of box 0, or the maximum
            for dim in range(3)  # only intersection if all dimensions overlap
        )

    @staticmethod
    def within_box(pt, box):
        """
        returns if point is in box, inclusive
        @param pt: point
        @param box:  box
        """
        return all(
            box[dim][0] <= pt[dim] and pt[dim] <= box[dim][1]
            for dim in range(3)  # only within if all dimensions overlap
        )

    @staticmethod
    def angle_diff(a0, a1):
        """
        returns angle difference of a0-a1, on [-pi,pi)

        returns a0-a1
        """
        diff = (a0 - a1)%(2*np.pi)  # on [0,2pi)
        if diff >= np.pi:
            diff = diff - 2*np.pi
        return diff

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

    def add_problem_model(self, handle):
        """
        adds model to check bounding box of more thorougly for collisions

        @param handle: model handle to add
        """
        self.problem_models[handle] = None

    def reset_problem_modles(self):
        """
        resets problem models
            for use when scene has moved, keeps the handles in dictionary, just resets pre-found bounding boxes
        """
        self.problem_models = {h: None for h in self.problem_models}

    def clear_problem_modles(self):
        """
        clears problem models
        """
        self.problem_models = dict()

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
        theta = np.arctan2(vec[1], vec[0])%(2*np.pi)
        phi = np.pi/2 - np.arctan2(vec[2], np.linalg.norm(vec[:2]))

        l_tant = int(l*phi/(np.pi))
        # is in range [0,l], needs to be clipped if phi=pi

        k_tant = int(k*theta/(2*np.pi))
        # will always be in range [0,k)
        return min(l - 1, l_tant), k_tant

    def _get_k_tant(self, vec, k):
        """
        gets k-tant of vec
            i.e. k=4 is equivalent to quadrants

        @param vec: vector to check
        @param k: divisions of theta to consider when using spherical coordinates
        @return: k-tant;
            i.e. return of j means in polar coords, vec is in range theta=2*pi*[j,j+1)/k
        """
        theta = np.arctan2(vec[1], vec[0])
        return self._get_k_tant_from_theta(theta, k=k)

    def _get_k_tant_from_theta(self, theta, k):
        """
        gets k-tant of vector at angle theta
            i.e. k=4 is equivalent to quadrants

        @param theta: angle to check
        @param k: divisions of theta to consider when using spherical coordinates
        @return: k-tant;
            i.e. return of j means in polar coords, theta is in range 2*pi*[j,j+1)/k
        """
        k_tant = int(k*(theta%(2*np.pi))/(2*np.pi))
        # will always be in range [0,k)
        return k_tant


class BlimpExperiment(Experiment):

    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100,
                 ):
        """
        experiments involving blimp swarms (controlled by ROS velocity controller)

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            scenePath=scenePath,
            sim=sim,
            simId=simId,
            wakeup=wakeup,
            sleeptime=sleeptime,
        )
        self.num_agents = num_agents
        self.msg_queue = msg_queue
        self.start_zone = start_zone
        self.modelPath = blimpPath
        self.agentData = dict()
        self.spawn_tries = spawn_tries

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        TOPIC_PRE_BLIMP = TOPIC_NAMES['TOPIC_PRE_BLIMP']
        TOPIC_CMD = TOPIC_NAMES['TOPIC_CMD']
        TOPIC_GLOBAL = TOPIC_NAMES['TOPIC_GLOBAL']
        TOPIC_ULTRA = TOPIC_NAMES['TOPIC_ULTRA']

        if not rclpy.ok():
            rclpy.init()
        self.agentData = dict()
        for i in range(self.num_agents):
            this_agent = dict()
            this_agent['agentHandle'] = self.spawnModel(self.modelPath, lambda: self.start_zone(i), self.spawn_tries)
            this_agent['agent_id'] = i

            unique = str(time.time()).replace('.', '_')
            NODE = rclpy.create_node('lta_' + str(self.simId) + '_' + str(i) + '_NODE_' + unique)
            this_agent['NODE'] = NODE

            cmd_topic = TOPIC_PRE_BLIMP + str(self.simId) + '_' + str(i) + TOPIC_CMD
            this_agent['cmd_topic'] = cmd_topic

            state_topic = TOPIC_PRE_BLIMP + str(self.simId) + '_' + str(i) + TOPIC_GLOBAL
            this_agent['state_topic'] = state_topic

            ultra_topic = TOPIC_PRE_BLIMP + str(self.simId) + '_' + str(i) + TOPIC_ULTRA
            this_agent['ultra_topic'] = ultra_topic

            vec_publisher = NODE.create_publisher(Twist, cmd_topic, self.msg_queue)
            this_agent['vec_publisher'] = vec_publisher

            callback = self.create_callback_twist(this_agent, 'state')
            state_subscriber = NODE.create_subscription(TwistStamped,
                                                        state_topic,
                                                        callback,
                                                        self.msg_queue)
            this_agent['state_subscriber'] = state_subscriber

            ultracallback = self.create_callback_float(this_agent, 'state')
            ultra_subscriber = NODE.create_subscription(Float64,
                                                        ultra_topic,
                                                        ultracallback,
                                                        self.msg_queue)
            this_agent['ultra_subscriber'] = ultra_subscriber

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(NODE)
            this_agent['executor'] = executor
            self.agentData[i] = this_agent

    def despawnThings(self):
        """
        to be run at end of each expiriment

        @note: FOR SOME REASON, it works better to not delete the nodes, and just leave them as warnings
        """
        return
        for agent_id in self.agentData:
            for sub_key in ('state_subscriber', 'ultra_subscriber'):
                self.agentData[agent_id]['NODE'].destroy_subscription(sub_key)
            for pub_key in ('vec_publisher',):
                self.agentData[agent_id]['NODE'].destroy_publisher(pub_key)
            self.agentData[agent_id]['NODE'].destroy_node()

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
            self.agentData[agent_id]['executor'].spin_once(timeout_sec=.01)

    def create_callback_twist(self, dictionary, key, state_keys=('x', 'y', 'z', 'w', 'DEBUG')):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_keys: keys to put x,y,z,w,DEBUG values
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
            dictionary[key][state_keys[4]] = 1.

        return callback

    def create_callback_float(self, dictionary, key, state_key='ultra', debug_key="ULTRA_DEBUG"):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_key: key to put float values
        @param debug_key: key to put debug stuff (currently whether callback is run)
        @return: returns callback function to be used in ROS subscriber
        """
        if key not in dictionary:
            dictionary[key] = dict()
        # default value of 0
        dictionary[key].update({state_key: 0., debug_key: 0.})

        def callback(msg):
            dictionary[key][state_key] = msg.data
            dictionary[key][debug_key] = 1.

        return callback

    ####################################################################################################################
    # Agent functions
    ####################################################################################################################

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

    def is_connected(self, agent_id):
        """
        returns if the agent state has been read by ROS

        @param agent_id: agent id

        @return: boolean on if state has been seen
        """
        s = self.get_state(agent_id=agent_id, spin=False)
        return bool(s["DEBUG"])

    def get_state(self, agent_id, spin=True):
        """
        returns state of agent

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
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
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
        return self._gen_get_neighbors(agent_id, lambda id0, id1: self.within_range(id0, id1, rng, spin=False), spin)

    def get_neighbors_3d_l_k_tant(self, agent_id, is_neigh, l=2, k=8, spin=True):
        """
        gets count of neighbors in each l,k-tant
            i.e. l=2, k=1 is equivalent to 'north hemisphere, south hemisphere'
            l=2 k=4 is equivalent to octants of a sphere

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^(l x k) array
        @return: return[i][j] specifies number of neighbors in the range phi=pi*[i,i+1)/l and theta=2*pi*[j,j+1)/k
        """
        neighbors = self._gen_get_neighbors(agent_id=agent_id, is_neigh=is_neigh, spin=spin)
        output = [[0 for _ in range(k)] for _ in range(l)]
        pos = self.get_position(agent_id, use_ultra=False, spin=False)
        for neigh_id in neighbors:
            neigh_pos = self.get_position(neigh_id, use_ultra=False, spin=False)
            vec = neigh_pos - pos
            l_tant, k_tant = self._get_l_k_tant(vec=vec, l=l, k=k)
            output[l_tant][k_tant] += 1
        return np.array(output)

    def get_neighbors_2d_k_tant(self, agent_id, is_neigh, k=8, spin=True):
        """
        gets count of neighbors in each k-tant
            i.e. k=4 is equivalent to quadrants of the xy plane

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
        @param k: divisions of theta to consider when using spherical coordinates
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^k array
        @return: return[i] specifies number of neighbors in the approximate direction ()

        @note: when getting neighbors with range, z direction is considered,
        """
        return self.get_neighbors_3d_l_k_tant(agent_id=agent_id, is_neigh=is_neigh, l=1, k=k, spin=spin)[0]

    def global_get_inv_dist_3d_l_k_tant(self, agent_id, is_neigh, l=2, k=8, min_dist=.01, spin=True):
        """
        gets inverse distance from nearest neighbor in each l,k-tant (0 if no neighbor)
            i.e. l=2, k=1 is equivalent to 'north hemisphere, south hemisphere'
            l=2 k=4 is equivalent to octants of a sphere

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
        @param min_dist: lowest distance to sense, to avoid division by 0
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^(l x k) array
        @return: return[i][j] specifies inverse distance to nearest neighbor
            in the range phi=pi*[i,i+1)/l and theta=2*pi*[j,j+1)/k
        """
        neighbors = self._gen_get_neighbors(agent_id=agent_id, is_neigh=is_neigh, spin=spin)
        output = [[0 for _ in range(k)] for _ in range(l)]
        pos = self.get_position(agent_id, use_ultra=False, spin=False)
        for neigh_id in neighbors:
            neigh_pos = self.get_position(neigh_id, use_ultra=False, spin=False)
            vec = neigh_pos - pos
            d = np.linalg.norm(vec)
            if d < min_dist:
                d = min_dist
            l_tant, k_tant = self._get_l_k_tant(vec=vec, l=l, k=k)
            output[l_tant][k_tant] = max(output[l_tant][k_tant], 1/d)
        return np.array(output)

    def global_get_inv_dist_2d_k_tant(self, agent_id, is_neigh, k=8, min_dist=.01, spin=True):
        """
        gets count of neighbors in each k-tant
            i.e. k=4 is equivalent to quadrants of the xy plane

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
        @param k: divisions of theta to consider when using spherical coordinates
        @param min_dist: lowest distance to sense, to avoid division by 0
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^k array
        @return: return[j] specifies distance to nearest neighbor in the range theta=2*pi*[j,j+1)/k

        @note: when getting neighbors with range, z direction is considered,
        """
        return self.global_get_inv_dist_3d_l_k_tant(agent_id=agent_id, is_neigh=is_neigh, l=1, k=k, min_dist=min_dist,
                                                    spin=spin)[0]

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def within_range(self, id0, id1, rng, spin=False):
        """
        checks whether id0 agent is within 'range' of id1 agent

        @param id0: agent 0 id
        @param id1: agent 1 id
        @param rng: R, range to check
        @param spin: whether to update each object before getting position
        @return: boolean, whether the two agents are close enough
        """
        pos1 = self.get_position(id0, use_ultra=False, spin=spin)
        pos2 = self.get_position(id1, use_ultra=False, spin=spin)
        return np.linalg.norm(pos1 - pos2) <= rng


class blimpTest(BlimpExperiment):
    def __init__(self, num_agents,
                 start_zone,
                 command=(0., 0., 0.),
                 scene_path=cage_arena_path,
                 blimp_path=narrow_blimp_path,
                 end_time=10,
                 simId=23000,
                 wakeup=None):
        super().__init__(num_agents, start_zone, scene_path, blimp_path, simId=simId, wakeup=wakeup)
        self.command = command
        self.end_time = end_time

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
            just moves towards next agent's position

        @return: boolean, whether or not experiment is done
        """
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, use_ultra=False, spin=True)
            self.move_agent(agent_id, self.command)
        return self.sim.getSimulationTime() > self.end_time

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        returns z position right now
        """
        s = []
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, use_ultra=False)
            s.append(pos[2])
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                print("ERROR DEBUG")
                return None
        return s


class AnkiExperiment(Experiment):

    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 ankiPath,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100,
                 ):
        """
        experiments involving anki swarms (controlled by ROS velocity controller)

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            scenePath=scenePath,
            sim=sim,
            simId=simId,
            wakeup=wakeup,
            sleeptime=sleeptime,
        )
        self.num_agents = num_agents
        self.msg_queue = msg_queue
        self.start_zone = start_zone
        self.modelPath = ankiPath
        self.agentData = dict()
        self.spawn_tries = spawn_tries
        self.anki_length = .08

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        TOPIC_PRE_ANKI = TOPIC_NAMES['TOPIC_PRE_ANKI']
        TOPIC_CMD = TOPIC_NAMES['TOPIC_CMD']
        TOPIC_GLOBAL = TOPIC_NAMES['TOPIC_GLOBAL']
        TOPIC_PROXIMITY = TOPIC_NAMES['TOPIC_PROXIMITY']

        if not rclpy.ok():
            rclpy.init()
        self.agentData = dict()
        positions = []
        for i in range(self.num_agents):
            this_agent = dict()
            # TODO: fix this mess
            this_agent['agentHandle'] = self.spawnModel(self.modelPath,
                                                        lambda: self.start_zone(i),
                                                        1,
                                                        orient_rng=lambda: (0, 0, (0, 2*np.pi)))
            for _ in range(self.spawn_tries):
                pos = np.array(self.sim.getObjectPosition(this_agent['agentHandle'], self.sim.handle_world))
                done = True
                for p2 in positions:
                    if np.linalg.norm(pos - p2) < self.anki_length:
                        done = False
                        break
                if done:
                    positions.append(pos)
                    break
                self.moveObject(this_agent['agentHandle'], lambda: self.start_zone(i),
                                orient_rng=lambda: (0, 0, (0, 2*np.pi)))

            this_agent['agent_id'] = i

            unique = str(time.time()).replace('.', '_')
            NODE = rclpy.create_node('anki_' + str(self.simId) + '_' + str(i) + '_NODE_' + unique)
            this_agent['NODE'] = NODE

            cmd_topic = TOPIC_PRE_ANKI + str(self.simId) + '_' + str(i) + TOPIC_CMD
            this_agent['cmd_topic'] = cmd_topic

            state_topic = TOPIC_PRE_ANKI + str(self.simId) + '_' + str(i) + TOPIC_GLOBAL
            this_agent['state_topic'] = state_topic

            prox_topic = TOPIC_PRE_ANKI + str(self.simId) + '_' + str(i) + TOPIC_PROXIMITY
            this_agent['prox_topic'] = prox_topic

            vec_publisher = NODE.create_publisher(Twist, cmd_topic, self.msg_queue)
            this_agent['vec_publisher'] = vec_publisher

            callback = self.create_callback_twist(this_agent, 'state')
            state_subscriber = NODE.create_subscription(TwistStamped,
                                                        state_topic,
                                                        callback,
                                                        self.msg_queue)
            this_agent['state_subscriber'] = state_subscriber

            proxcallback = self.create_callback_float(this_agent, 'state')
            prox_subscriber = NODE.create_subscription(Float64,
                                                       prox_topic,
                                                       proxcallback,
                                                       self.msg_queue)
            this_agent['prox_subscriber'] = prox_subscriber

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(NODE)
            this_agent['executor'] = executor
            self.agentData[i] = this_agent

    def despawnThings(self):
        """
        to be run at end of each expiriment

        @note: FOR SOME REASON, it works better to not delete the nodes, and just leave them as warnings
        """
        return
        for agent_id in self.agentData:
            for sub_key in ('state_subscriber', 'prox_subscriber'):
                self.agentData[agent_id]['NODE'].destroy_subscription(sub_key)
            for pub_key in ('vec_publisher',):
                self.agentData[agent_id]['NODE'].destroy_publisher(pub_key)
            self.agentData[agent_id]['NODE'].destroy_node()

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
            self.agentData[agent_id]['executor'].spin_once(timeout_sec=.01)

    def create_callback_twist(self, dictionary, key, state_keys=('x', 'y', 'z', 'w', 'DEBUG')):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_keys: keys to put x,y,z,w,DEBUG values
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
            dictionary[key][state_keys[4]] = 1.

        return callback

    def create_callback_float(self, dictionary, key, state_key='proximity', debug_key="PROX_DEBUG"):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_key: key to put float values
        @param debug_key: key to put debug stuff (currently whether callback is run)
        @return: returns callback function to be used in ROS subscriber
        """
        if key not in dictionary:
            dictionary[key] = dict()
        # default value of 0
        dictionary[key].update({state_key: 0., debug_key: 0.})

        def callback(msg):
            dictionary[key][state_key] = msg.data
            dictionary[key][debug_key] = 1.

        return callback

    ####################################################################################################################
    # Agent functions
    ####################################################################################################################
    def move_agent(self, agent_id, vec):
        """
        publishes a vector to an agent
            (currently publishes a velocity goal, and LUA controls in blimpNew.lua takes care of rest)

        @param agent_id: agent id to publish to
        @param vec: vector to publish
            order right now is left wheel, right wheel, lift, head

        @note: (left wheel V, right wheel V, lift, lift boolean, head tile, head tilt boolean)
            is the vector expected by coppelia model
        """
        if agent_id not in self.agentData:
            raise Exception("attempted to move agent that does not exist: id of " + str(agent_id))
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])

        msgTwist.linear.y = float(vec[1])

        if len(vec) == 4:
            msgTwist.linear.z = float(vec[2])
            msgTwist.angular.x = float(1)

            msgTwist.angular.y = float(vec[3])
            msgTwist.angular.z = float(1)

        self.agentData[agent_id]['vec_publisher'].publish(msgTwist)

    def move_agent_thrust_rot(self, agent_id, vec):
        """
        OLD VERSION
        publishes a vector to an agent
            (currently publishes a velocity goal, and LUA controls in blimpNew.lua takes care of rest)

        @param agent_id: agent id to publish to
        @param vec: vector to publish
            order right now is thrust, turn, lift, head

        @note: (thrust, lift, lift boolean, head tilt, head tilt boolean, rotation)
            is the vector expected by coppelia model
        """
        if agent_id not in self.agentData:
            raise Exception("attempted to move agent that does not exist: id of " + str(agent_id))
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])

        msgTwist.angular.z = float(vec[1])

        if len(vec) == 4:
            msgTwist.linear.y = float(vec[2])
            msgTwist.linear.z = float(1)

            msgTwist.angular.x = float(vec[3])
            msgTwist.angular.y = float(1)

        self.agentData[agent_id]['vec_publisher'].publish(msgTwist)

    def get_state(self, agent_id, spin=True):
        """
        returns state of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: dictionary
        @return: state of agent
        """
        if spin:
            self.spin([agent_id])
        return self.agentData[agent_id]['state']

    def get_position(self, agent_id, spin=True):
        """
        returns position of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: R^2 numpy array
        @return: position of agent
        """
        s = self.get_state(agent_id, spin=spin)
        return np.array((s['x'], s['y']))

    def get_head(self, agent_id, spin=True):
        """
        returns heading of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: R
        @return: heading of agent
        """
        s = self.get_state(agent_id, spin=spin)
        return s['w']

    def get_prox(self, agent_id, spin=True):
        """
        returns proximity sensor of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: R^2 numpy array
        @return: position of agent
        """
        s = self.get_state(agent_id, spin=spin)
        return s['proximity']

    def _gen_get_neighbors(self, agent_id, is_neigh, spin=False):
        """
        gets neighbors of an agent, general implementation

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
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
        return self._gen_get_neighbors(agent_id, lambda id0, id1: self.within_range(id0, id1, rng, spin=False), spin)

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def within_range(self, id0, id1, rng, spin=False):
        """
        checks whether id0 agent is within 'range' of id1 agent

        @param id0: agent 0 id
        @param id1: agent 1 id
        @param rng: R, range to check
        @param spin: whether to update each object before getting position
        @return: boolean, whether the two agents are close enough
        """
        pos1 = self.get_position(id0, spin=spin)
        pos2 = self.get_position(id1, spin=spin)
        return np.linalg.norm(pos1 - pos2) <= rng

    def get_neighbors_k_tant(self, agent_id, is_neigh, k=8, spin=True):
        """
        gets count of neighbors in each k-tant
            i.e. k=4 is equivalent to quadrants

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
        @param k: divisions of theta to consider when using spherical coordinates
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^k array
        @return: return[i] specifies number of neighbors in the range theta=2*pi*[j,j+1)/k
        """
        neighbors = self._gen_get_neighbors(agent_id=agent_id, is_neigh=is_neigh, spin=spin)
        output = [0 for _ in range(k)]
        pos = self.get_position(agent_id, spin=False)
        head = self.get_head(agent_id, spin=False)
        for neigh_id in neighbors:
            neigh_pos = self.get_position(neigh_id, spin=False)
            vec = neigh_pos - pos
            theta_global = np.arctan2(vec[1], vec[0])%(2*np.pi)
            theta_local = theta_global - head
            k_tant = self._get_k_tant_from_theta(theta_local, k=k)
            output[k_tant] += 1
        return np.array(output)

    def local_get_inv_dist_k_tant(self, agent_id, is_neigh, k=8, min_dist=.01, spin=True):
        """
        gets inverse distance from nearest neighbor in each k-tant (0 if no neighbor)
            i.e. k=4 is equivalent to quadrants

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
        @param k: divisions of theta to consider when using spherical coordinates
        @param min_dist: lowest distance to sense, to avoid division by 0
        @param spin: whether to update all agents before checking neighbors
        @rtype: N^k array
        @return: return[j] specifies distance to nearest neighbor in the range theta=2*pi*[j,j+1)/k
        """
        neighbors = self._gen_get_neighbors(agent_id=agent_id, is_neigh=is_neigh, spin=spin)
        output = [0 for _ in range(k)]
        pos = self.get_position(agent_id, spin=False)
        head = self.get_head(agent_id, spin=False)
        for neigh_id in neighbors:
            neigh_pos = self.get_position(neigh_id, spin=False)
            vec = neigh_pos - pos
            theta_global = np.arctan2(vec[1], vec[0])%(2*np.pi)
            theta_local = theta_global - head

            d = np.linalg.norm(vec)
            if d < min_dist:
                d = min_dist
            k_tant = self._get_k_tant_from_theta(theta_local, k=k)
            output[k_tant] = max(output[k_tant], 1/d)
        return np.array(output)


class ankiTest(AnkiExperiment):
    def __init__(self, num_agents,
                 start_zone,
                 command,
                 scene_path=anki_arena_path,
                 blimp_path=anki_path,
                 end_time=10,
                 simId=23000,
                 wakeup=None):
        super().__init__(num_agents, start_zone, scene_path, blimp_path, simId=simId, wakeup=wakeup)
        self.command = command
        self.end_time = end_time

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
            just moves towards next agent's position

        @return: boolean, whether or not experiment is done
        """
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, spin=True)
            self.move_agent(agent_id, self.command)
        return self.sim.getSimulationTime() > self.end_time

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        returns z position right now
        """
        s = []
        for agent_id in self.agentData:
            pos = self.get_position(agent_id)
            s.append(pos[1])
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                print("ERROR DEBUG")
                return None
        return s


class CopterExperiment(Experiment):

    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 copterPath,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100,
                 ):
        """
        experiments involving copter swarms (controlled by ROS something controller)

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            scenePath=scenePath,
            sim=sim,
            simId=simId,
            wakeup=wakeup,
            sleeptime=sleeptime,
        )
        self.num_agents = num_agents
        self.msg_queue = msg_queue
        self.start_zone = start_zone
        self.modelPath = copterPath
        self.agentData = dict()
        self.spawn_tries = spawn_tries

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        TOPIC_PRE_QUAD = TOPIC_NAMES['TOPIC_PRE_QUAD']
        TOPIC_CMD = TOPIC_NAMES['TOPIC_CMD']
        TOPIC_GLOBAL = TOPIC_NAMES['TOPIC_GLOBAL']

        if not rclpy.ok():
            rclpy.init()
        self.agentData = dict()
        for i in range(self.num_agents):
            this_agent = dict()
            this_agent['agentHandle'] = self.spawnModel(self.modelPath,
                                                        lambda: self.start_zone(i), self.spawn_tries,
                                                        orient_rng=lambda: (0, 0, 90))
            this_agent['agent_id'] = i

            unique = str(time.time()).replace('.', '_')
            NODE = rclpy.create_node('anki_' + str(self.simId) + '_' + str(i) + '_NODE_' + unique)
            this_agent['NODE'] = NODE

            cmd_topic = TOPIC_PRE_QUAD + str(self.simId) + '_' + str(i) + TOPIC_CMD
            this_agent['cmd_topic'] = cmd_topic

            state_topic = TOPIC_PRE_QUAD + str(self.simId) + '_' + str(i) + TOPIC_GLOBAL
            this_agent['state_topic'] = state_topic

            vec_publisher = NODE.create_publisher(Twist, cmd_topic, self.msg_queue)
            this_agent['vec_publisher'] = vec_publisher

            callback = self.create_callback_twist(this_agent, 'state')
            state_subscriber = NODE.create_subscription(TwistStamped,
                                                        state_topic,
                                                        callback,
                                                        self.msg_queue)
            this_agent['state_subscriber'] = state_subscriber

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(NODE)
            this_agent['executor'] = executor
            self.agentData[i] = this_agent

    def despawnThings(self):
        """
        to be run at end of each expiriment

        @note: FOR SOME REASON, it works better to not delete the nodes, and just leave them as warnings
        """
        return
        for agent_id in self.agentData:
            for sub_key in ('state_subscriber',):
                self.agentData[agent_id]['NODE'].destroy_subscription(sub_key)
            for pub_key in ('vec_publisher',):
                self.agentData[agent_id]['NODE'].destroy_publisher(pub_key)
            self.agentData[agent_id]['NODE'].destroy_node()

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
            self.agentData[agent_id]['executor'].spin_once(timeout_sec=.01)

    def create_callback_twist(self, dictionary, key, state_keys=('x', 'y', 'z', 'r', 'p', 'w', 'DEBUG')):
        """
        creates a callback that updates the "key" element of "dictionary" with the twist state

        @param dictionary: dictionary to update
        @param key: key in dictionary to update
        @param state_keys: keys to put x,y,z,roll,pitch,yaw,DEBUG values
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
            dictionary[key][state_keys[3]] = msg.twist.angular.x
            dictionary[key][state_keys[4]] = msg.twist.angular.y
            dictionary[key][state_keys[5]] = msg.twist.angular.z
            dictionary[key][state_keys[6]] = 1.

        return callback

    ####################################################################################################################
    # Agent functions
    ####################################################################################################################

    def move_agent(self, agent_id, vec):
        """
        publishes a vector to an agent
            (currently publishes a velocity goal, and LUA controls in blimpNew.lua takes care of rest)

        @param agent_id: agent id to publish to
        @param vec: vector to publish
            order right now is xyz velocity and heading

        @note: currently using just the linear part of twist message,
            can use orientation for other stuff if we update blimpNew.lua
        """
        if agent_id not in self.agentData:
            raise Exception("attempted to move agent that does not exist: id of " + str(agent_id))
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])
        msgTwist.linear.y = float(vec[1])
        msgTwist.linear.z = float(vec[2])

        msgTwist.angular.z = float(vec[3])

        self.agentData[agent_id]['vec_publisher'].publish(msgTwist)

    def get_state(self, agent_id, spin=True):
        """
        returns state of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: dictionary
        @return: state of agent
        """
        if spin:
            self.spin([agent_id])
        return self.agentData[agent_id]['state']

    def get_position(self, agent_id, spin=True):
        """
        returns position of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: R^2 numpy array
        @return: position of agent
        """
        s = self.get_state(agent_id, spin=spin)
        return np.array((s['x'], s['y'], s['z']))

    def _gen_get_neighbors(self, agent_id, is_neigh, spin=False):
        """
        gets neighbors of an agent, general implementation

        @param agent_id: agent id
        @param is_neigh: agent_id0 x agent_id1 -> bool; returns if agent 1 is a neighbor of agent 0
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
        return self._gen_get_neighbors(agent_id, lambda id0, id1: self.within_range(id0, id1, rng, spin=False), spin)

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def within_range(self, id0, id1, rng, spin=False):
        """
        checks whether id0 agent is within 'range' of id1 agent

        @param id0: agent 0 id
        @param id1: agent 1 id
        @param rng: R, range to check
        @param spin: whether to update each object before getting position
        @return: boolean, whether the two agents are close enough
        """
        pos1 = self.get_position(id0, spin=spin)
        pos2 = self.get_position(id1, spin=spin)
        return np.linalg.norm(pos1 - pos2) <= rng


class quadTest(CopterExperiment):
    def __init__(self, num_agents,
                 start_zone,
                 command,
                 scene_path=cage_arena_path,
                 copterPath=quad_path,
                 end_time=10,
                 simId=23000,
                 wakeup=None):
        super().__init__(num_agents, start_zone, scene_path, copterPath=copterPath, simId=simId, wakeup=wakeup)
        self.command = command
        self.end_time = end_time

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
            just moves towards next agent's position

        @return: boolean, whether or not experiment is done
        """
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, spin=True)

            self.move_agent(agent_id, self.command)
        return self.sim.getSimulationTime() > self.end_time

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        returns z position right now
        """
        s = []
        for agent_id in self.agentData:
            pos = self.get_position(agent_id)
            s.append(pos[1])
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                print("ERROR DEBUG")
                return None
        return s


if __name__ == "__main__":
    bb = blimpTest(10,
                   lambda i: ((-5, 5), (-5, 5), (1, 5)),
                   command=(0, 0, .1),  # (x, y, z) velocity
                   wakeup=[COPPELIA_WAKEUP])
    bb.run_exp()
    bb.kill()

    aa = ankiTest(1,
                  lambda i: ((-.35, .35), (-.35, .35), .035),
                  command=(.22, -.22),  # (left wheel, right wheel, arm lift, head tilt)
                  wakeup=[COPPELIA_WAKEUP])
    aa.run_exp()
    aa.kill()

    qq = quadTest(5,
                  lambda i: (i - .35, 0, 1),
                  command=(0, 0, .1, .1),  # (x, y, z, rotation) force
                  wakeup=[COPPELIA_WAKEUP])
    qq.run_exp()
    qq.kill()
