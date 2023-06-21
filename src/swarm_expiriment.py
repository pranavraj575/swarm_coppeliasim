import subprocess, psutil
import time, os, sys
import rclpy
import numpy as np

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

frictionless_wall_path = os.path.join(DIR, 'scenes', 'FrictionlessWallClimb.ttt')
empty_path = os.path.join(DIR, 'scenes', 'empty.ttt')
narrow_blimp_path = os.path.join(DIR, 'ros_ctrl_models', 'blimp_narrow.ttm')
COPPELIA_WAKEUP = '/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh'


class Experiment:
    def __init__(self,
                 scenePath,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=.01,
                 ):
        """
        General class that intializes and runs a repeatable experiment, returning results

        @param scenePath: path to the scene to load
        @param sim: zqm simulator api, if None, than makes its own
        @param simId: simulator id, used to pass messages to correct topics
        @param wakeup: list of commands to run on initialization (i.e. start coppeliasim)
        @param sleeptime: time to wait after important commands (start/stop/pause simulation)
        """
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

        @return: can be anything
        """
        raise NotImplementedError()


# NODE=None

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
    def spawnBlimp(self, modelPath, pos_rng, spawn_tries, orientation=None):
        """
        spawns a model in a certian area

        @param modelPath: path to .ttm model to spawn
        @param pos_rng: ()->(R U R^2)^3, range to spawn into
            (if single value, this is the value for the respective coordinate)
            (if tuple, than uniformly chooses from (low,high))
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        @param orientation: R^3, orientation to spawn into, None if default model orientation
        @return: handle of model spawned
        """
        agentHandle = self.sim.loadModel(os.path.abspath(os.path.expanduser(modelPath)))
        for _ in range(spawn_tries):
            Pos = []
            rng = pos_rng()
            for k in range(3):
                try:
                    Pos.append(np.random.uniform(rng[k][0], rng[k][1]))
                except:
                    Pos.append(float(rng[k]))
            self.sim.setObjectPosition(agentHandle, -1, Pos)
            if orientation is not None:
                self.sim.setObjectOrientation(agentHandle, -1, [float(o) for o in orientation])
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
            this_agent['agentHandle'] = self.spawnBlimp(self.modelPath, lambda: self.start_zone(i), self.spawn_tries)
            this_agent['agent_id'] = i

            unique = str(time.time()).replace('.', '_')
            NODE = rclpy.create_node('lta_' + str(self.simId) + '_' + str(i) + '_NODE_' + unique)
            this_agent['NODE'] = NODE

            cmd_topic = TOPIC_PRE + str(self.simId) + '_' + str(i) + TOPIC_CMD
            this_agent['cmd_topic'] = cmd_topic

            state_topic = TOPIC_PRE + str(self.simId) + '_' + str(i) + TOPIC_GLOBAL
            this_agent['state_topic'] = state_topic
            ultra_topic = TOPIC_PRE + str(self.simId) + '_' + str(i) + TOPIC_ULTRA
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

    def get_neighbors_3d_l_k_ant(self, agent_id, rng, l=2, k=8, spin=True):
        """
        gets count of neighbors in each l,k-ant
            i.e. l=2, k=1 is equivalent to 'north hemisphere, south hemisphere'
            l=2 k=4 is equivalent to octants of a sphere

        @param agent_id: agent id
        @param rng: range for which agents count as neighbors
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
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
        return np.array(output)

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
                                             l=1,
                                             k=k,
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
    def __init__(self, num_agents,
                 start_zone,
                 command=(0., 0., 0.),
                 scene_path=empty_path,
                 blimp_path=narrow_blimp_path,
                 simId=23000,
                 wakeup=None):
        super().__init__(num_agents, start_zone, scene_path, blimp_path, simId=simId, wakeup=wakeup)
        self.command = command

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
        return False

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
    bb = blimpTest(10, lambda i: ((-5, 5), (-5, 5), (1, 5)), command=(0, 0, .1), wakeup=[COPPELIA_WAKEUP])
    bb.run_exp()
