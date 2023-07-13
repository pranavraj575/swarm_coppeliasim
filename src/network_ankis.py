from src.swarm_expiriment import *
import numpy as np


class ankiNet(AnkiExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 ankiPath,
                 networkfn,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=1,
                 ):
        """
        experiement of anki swarm each controlled by a neural network

        @param num_agents: number of ankis in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the anki number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param networkfn: neural network function to call for anki to act
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            num_agents=num_agents,
            start_zone=start_zone,
            scenePath=scenePath,
            ankiPath=ankiPath,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.network = networkfn
        self.last_time = 0

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: np array with correct dimensions
        """
        raise NotImplementedError()

    def get_vec_from_net_ouput(self, output, agent_id):
        """
        given network output, transform into control vector
            useful if we want edited controls (i.e. max speed or more drastic stuff like using position waypoints)

        @param output: network output
        @param agent_id: could be important for subclasses
        @return: control vector, probably R^3 encoding velocity goal
        """
        raise NotImplementedError()

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        self.spin()
        for agent_id in self.agentData:
            z = self.network(self.get_network_input(agent_id))
            vec = self.get_vec_from_net_ouput(z, agent_id)
            self.move_agent(agent_id, vec)
        t = self.sim.getSimulationTime()
        # print('cycle time:',t-self.last_time,end='\r')
        self.last_time = t
        return self.end_test()

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        raise NotImplementedError()


class ecosystemAnkiNet(AnkiExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 ankiPath,
                 networkfns,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=1,
                 ):
        """
        experiement of anki swarm each controlled by a different neural network for each anki
            same as above, except the networkfns argument

        @param num_agents: number of ankis in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the anki number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param networkfns: agentid -> neural network function to call for anki to act
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            num_agents=num_agents,
            start_zone=start_zone,
            scenePath=scenePath,
            ankiPath=ankiPath,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.networks = networkfns
        self.last_time = 0

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: np array with correct dimensions
        """
        raise NotImplementedError()

    def get_vec_from_net_ouput(self, output, agent_id):
        """
        given network output, transform into control vector
            useful if we want edited controls (i.e. max speed or more drastic stuff like using position waypoints)

        @param output: network output
        @param agent_id: could be important for subclasses
        @return: control vector, probably R^3 encoding velocity goal
        """
        raise NotImplementedError()

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        self.spin()
        for agent_id in self.agentData:
            network = self.networks(agent_id)
            z = network(self.get_network_input(agent_id))
            vec = self.get_vec_from_net_ouput(z, agent_id)
            self.move_agent(agent_id, vec)
        t = self.sim.getSimulationTime()
        # print('cycle time:',t-self.last_time,end='\r')
        self.last_time = t
        return self.end_test()

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        raise NotImplementedError()


class thrust_rot_anki(ankiNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 ankiPath,
                 networkfn,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=1):
        """
        each anki returns a thrust vector and a rotation

        @param num_agents: number of ankis in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the anki number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param networkfn: neural network function call for anki to act
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            num_agents=num_agents,
            start_zone=start_zone,
            scenePath=scenePath,
            ankiPath=ankiPath,
            networkfn=networkfn,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_vec_from_net_ouput(self, output, agent_id):
        """
        given network output, transform into control vector
            useful if we want edited controls (i.e. max speed or more drastic stuff like using position waypoints)

        @param output: network output
        @return: control vector, probably R^3 encoding velocity goal
        """
        th_rot = np.array(output).flatten()  # this is in ([0,1]^2)
        th_rot = th_rot*2 - 1  # cast into [-1,1]^2
        return th_rot


class th_rot_angle_anki(thrust_rot_anki):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 ankiPath,
                 networkfn,
                 angle_goal,
                 end_time=10,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=1):
        """
        anki only sees xy coordinates, rewarded for mean closeness to origin

        @param num_agents: number of ankis in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the anki number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param networkfn: neural network function call for anki to act
        @param angle_goal: heading angle to aim at
        @param end_time: time it takes for experiment to end
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            num_agents=num_agents,
            start_zone=start_zone,
            scenePath=scenePath,
            ankiPath=ankiPath,
            networkfn=networkfn,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.end_time = end_time
        self.angle_goal = angle_goal

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^3 np array
        """
        pos = self.get_position(agent_id, spin=True)
        head = self.get_head(agent_id, spin=False)
        print(head)
        return np.array([pos[0], pos[1], head])

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: negative average distance from origin
        """
        s = []
        for agent_id in self.agentData:
            head = self.get_head(agent_id)

            s.append(-abs(self.angle_diff(self.angle_goal, head)))
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time

    @staticmethod
    def angle_diff(a0, a1):
        """
        returns angle difference between two angles on [-pi,pi]

        returns a0-a1
        """
        diff = (a0 - a1)%(2*np.pi)
        if diff > np.pi:
            diff = diff - 2*np.pi
        return diff


class k_tant_anki_area_coverage(thrust_rot_anki):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 ankiPath,
                 networkfn,
                 bounds=((-.45, .6), (-.5, .5)),
                 end_time=10,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=1):
        """
        maximize deployment entropy given proximity sensor and k-tant information

        @param num_agents: number of ankis in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the anki number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param ankiPath: path to anki for spawning
        @param networkfn: neural network function call for anki to act
        @param bounds: (R x R)^2, x and y bounds of the area to cover
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(num_agents=num_agents,
                         start_zone=start_zone,
                         scenePath=scenePath,
                         ankiPath=ankiPath,
                         networkfn=networkfn,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries,
                         )
        self.bounds = bounds
        self.dimension_split = int(self.num_agents**(1/2))
        self.end_time = end_time

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^k np array
        """

        k_tant = self.local_get_inv_dist_k_tant(agent_id, is_neigh=lambda id0, id1: True, k=8, spin=True)
        prox = self.get_prox(agent_id, spin=False)
        return np.concatenate(([prox], k_tant))

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: deployment entropy
            evolution/papers/Persistent Area Coverage for ...
        """
        entropy = 0
        boxes = np.array([[0 for _ in range(self.dimension_split)] for _ in range(self.dimension_split)])
        xbound, ybound = self.bounds
        for agent_id in self.agentData:
            xy = self.get_position(agent_id)
            x, y = xy
            xbox = self.dimension_split*(x - xbound[0])/(xbound[1] - xbound[0])
            ybox = self.dimension_split*(y - ybound[0])/(ybound[1] - ybound[0])

            xbox = np.clip(xbox, 0, self.dimension_split - .5)
            ybox = np.clip(ybox, 0, self.dimension_split - .5)
            boxes[int(xbox), int(ybox)] += 1

            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None

        for x in boxes:
            for xy in x:
                p = xy/self.num_agents
                if p > 0:
                    entropy += p*np.log(1/p)
        return entropy

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time
