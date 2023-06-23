from src.swarm_expiriment import *
import numpy as np


class blimpNet(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100,
                 ):
        """
        experiement of blimp swarm each controlled by a neural network

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
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
            blimpPath=blimpPath,
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


class xyBlimp(blimpNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        each blimp returns an xy vector to go to
            height is always kept at 'height_range', using ultrasound if use_ultra is true

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param height_factor: factor to multiply height adjust by
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
            blimpPath=blimpPath,
            networkfn=networkfn,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.height_range = height_range
        self.use_ultra = use_ultra
        self.height_factor = height_factor

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
        xy = np.array(output).flatten()  # this is in ([0,1]^2)
        xy = xy*2 - 1  # cast into [-1,1]^2
        z = self.get_position(agent_id, use_ultra=self.use_ultra, spin=True)[2]
        h_adj = 0.
        if z < self.height_range[0]:
            h_adj = self.height_range[0] - z
        elif z > self.height_range[1]:
            h_adj = self.height_range[1] - z
        return np.concatenate((xy, np.array([h_adj*self.height_factor])))


class xy_zero_Blimp(xyBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 end_time=10,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        blimp only sees xy coordinates, rewarded for mean closeness to origin

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param end_time: time it takes for experiment to end
        @param height_factor: factor to multiply height adjust by
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
            blimpPath=blimpPath,
            networkfn=networkfn,
            height_range=height_range,
            use_ultra=use_ultra,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.end_time = end_time

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^2 np array
        """
        pos = self.get_position(agent_id, use_ultra=self.use_ultra, spin=True)[:2]
        return pos.reshape((2, 1))

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
            pos = self.get_position(agent_id, use_ultra=False)[:2]
            s.append(-np.linalg.norm(pos))
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                raise Exception("ERROR DEBUG")
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time


class xyzBlimp(blimpNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        each blimp only sees xyz coordinates, and returns an xyz vector to go to

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
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
            blimpPath=blimpPath,
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
        @param agent_id: could be important for subclasses
        @return: control vector, probably R^3 encoding velocity goal
        """
        xyz = np.array(output).flatten()  # this is in ([0,1]^3)
        xyz = xyz*2 - 1  # cast into [-1,1]^3
        return xyz


class xyz_zero_Blimp(xyzBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 end_time=10,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        xyz sample that is rewarded for closeness to origin

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
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
            blimpPath=blimpPath,
            networkfn=networkfn,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.end_time = end_time

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^3 np array
        """
        pos = self.get_position(agent_id, use_ultra=True, spin=True)
        return pos.reshape((3, 1))

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
            pos = self.get_position(agent_id, use_ultra=False)
            s.append(-np.linalg.norm(pos))
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                raise Exception("ERROR DEBUG")
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment
        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time


class l_k_tant_clump_blimp(blimpNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 end_time=10,
                 l=3,
                 k=4,
                 rng=2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100,
                 ):
        """
        each blimp sees its neighboring blimps, by seeing how many are in each l,k-tant
            viewed in spherical coordinates, l is the divisions of phi and k is the divisions of theta
            for example, l=2, k=4 would be the octants of a sphere
            rng is the range of neighbors that the blimp sees

        The blimps are rewarded for being close together

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
        @param end_time: time it takes for experiment to end
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
        @param rng: range for which agents count as neighbors
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
            blimpPath=blimpPath,
            networkfn=networkfn,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.end_time = end_time
        self.l = l
        self.k = k
        self.rng = rng  # note we can do better than this, as this allows agents to see through walls

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """
        l_k_tant = self.get_neighbors_3d_l_k_ant(agent_id=agent_id,
                                                 is_neigh=lambda id1, id2: self.within_range(id1=id1,
                                                                                             id2=id2,
                                                                                             rng=self.rng,
                                                                                             spin=False),
                                                 k=self.k,
                                                 l=self.l,
                                                 spin=True)
        return l_k_tant.reshape((-1, 1))

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: negative average distance between agents
        """
        s = []
        ids = list(self.agentData.keys())
        for i, id1 in enumerate(ids):
            for id2 in ids[i + 1:]:
                pos1 = self.get_position(id1, use_ultra=False)
                pos2 = self.get_position(id2, use_ultra=False)
                s.append(-np.linalg.norm(pos1 - pos2))
                # negative, we want to minimize distance between
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time
