from src.swarm_expiriment import *
import numpy as np


class blimpFSM(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
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
        self.solns = {agent_id: soln.clone() for agent_id in self.agentData}
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
            obs = self.get_network_input(agent_id)
            z = self.solns[agent_id].policy(obs)
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


class FSMxyBlimp(blimpFSM):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
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
            soln=soln,
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
        z = self.get_position(agent_id, use_ultra=self.use_ultra, spin=False)[2]
        h_adj = 0.
        if z < self.height_range[0]:
            h_adj = self.height_range[0] - z
        elif z > self.height_range[1]:
            h_adj = self.height_range[1] - z
        return np.concatenate((xy, np.array([h_adj*self.height_factor])))


class FSMxy_zero_Blimp(FSMxyBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
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
            soln=soln,
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
                return None
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time


class FSMk_tant_wall_climb_blimp(FSMxyBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 height_range,
                 use_ultra,
                 end_time,
                 rng,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        blimp sees number of neighbors on each octant, rewarded for climbing wall

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param end_time: time it takes for experiment to end
        @param rng: range to detect neighbors
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
            soln=soln,
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
        self.rng = rng

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """

        k_tant = self.get_neighbors_2d_k_tant(agent_id,
                                              is_neigh=lambda id0, id1: self.within_range(id0, id1, rng=self.rng), k=8,
                                              spin=True)
        return k_tant.reshape((-1, 1))

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: number of blimps over wall (negative average proximity to wall if none made it over)
        """
        over = 0
        best = float('inf')
        for agent_id in self.agentData:
            x = self.get_position(agent_id, use_ultra=False)[0]
            over += int(x < 0)
            best = min(best, x)
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None
        return float(over) if over > 0 else -best

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time


class FSMdist_k_tant_wall_climb_blimp(FSMk_tant_wall_climb_blimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 height_range,
                 use_ultra,
                 end_time,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        blimp sees the closest neighbor on each octant, rewarded for climbing wall

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
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
            soln=soln,
            height_range=height_range,
            use_ultra=use_ultra,
            end_time=end_time,
            rng=float('inf'),
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries,
        )

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """

        k_tant = self.global_get_inv_dist_2d_k_tant(agent_id, is_neigh=lambda id0, id1: True, k=8, spin=True)
        return k_tant.reshape((-1, 1))


class FSMk_tant_area_coverage(FSMxyBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 height_range,
                 use_ultra,
                 obstacles,
                 obstacle_height,
                 obstacle_paths,
                 end_time,
                 bounds,
                 k=8,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100
                 ):
        """
        blimp sees the closest neighbor on each octant, rewarded for closeness to randomly generated points

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param obstacles: number of obstacles to randomly spawn in
        @param obstacle_height: height to spawn in obstacles
        @param obstacle_paths: paths to obstacles to spawn in, list chosen from uniformly
        @param end_time: time it takes for experiment to end
        @param bounds: (RxR)^2, x bounds and y bounds to test for the area covered
            the goal function will uniformly choose some points in this area and rate the blimps based on closeness
        @param k: number of divisions to split sensing
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
            soln=soln,
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
        self.bounds = bounds
        self.k = k
        self.dimension_split = int(num_agents**(1/2))
        self.obstacles = obstacles
        self.obstacle_height = obstacle_height
        self.obstacle_paths = obstacle_paths
        self.obs_handles = None

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        self.obs_handles = []
        for _ in range(self.obstacles):
            obs = self.obstacle_paths[np.random.randint(0, len(self.obstacle_paths))]
            hand = self.spawnModel(obs,
                                   lambda: np.concatenate(
                                       (self.sample_from_bounds(1).flatten(), [self.obstacle_height])),
                                   spawn_tries=1,
                                   orient_rng=lambda: (0, 0, (0, 2*np.pi))
                                   )
            self.obs_handles.append(hand)
            self.add_problem_model(hand)
        super().spawnThings()

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^k np array
        """

        k_tant = self.global_get_inv_dist_2d_k_tant(agent_id, is_neigh=lambda id0, id1: True, k=self.k, spin=True)
        return k_tant.reshape((-1, 1))

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
            xy = self.get_position(agent_id, use_ultra=False)[:2]
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

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def sample_from_bounds(self, n):
        """
        samples uniformly from self.bounds
        @param n: number of samples to take
        @return: R^2 random sample
        """
        return np.random.uniform([b[0] for b in self.bounds], [b[1] for b in self.bounds], (n, len(self.bounds)))


class FSMk_tant_wall_sense_area_coverage(FSMk_tant_area_coverage):

    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 height_range,
                 use_ultra,
                 obstacles,
                 obstacle_height,
                 obstacle_paths,
                 end_time,
                 bounds,
                 k=8,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        blimp sees the closest neighbor on each octant, as well as bounds

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param obstacles: number of obstacles to randomly spawn in
        @param obstacle_height: height to spawn in obstacles
        @param obstacle_paths: paths to obstacles to spawn in, list chosen from uniformly
        @param end_time: time it takes for experiment to end
        @param bounds: (RxR)^2, x bounds and y bounds to test for the area covered
            the goal function will uniformly choose some points in this area and rate the blimps based on closeness
        @param k: number of divisions to split sensing
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
            soln=soln,
            height_range=height_range,
            use_ultra=use_ultra,
            obstacles=obstacles,
            obstacle_paths=obstacle_paths,
            obstacle_height=obstacle_height,
            end_time=end_time,
            bounds=bounds,
            k=k,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries,
        )

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^k np array
        """

        neighbor_k_tant = self.global_get_inv_dist_2d_k_tant(agent_id, is_neigh=lambda id0, id1: True, k=self.k,
                                                             spin=True)
        bounds_k_tant = self.get_bounds_k_tant(self.get_position(agent_id, spin=False), k=self.k)
        k_tant = np.max((neighbor_k_tant, bounds_k_tant), axis=0)
        return k_tant.reshape((-1, 1))

    def get_bounds_k_tant(self, pos, k, min_dist=.01):
        """
        gets the inverse distance to nearest bound for each k-tant

        @param pos: position to start from
        @param k: k-tants
        @return R^k array with inverse distance to closest bound on each k-tant
        """
        k_tant = np.zeros(k)
        xbound, ybound = self.bounds
        for i in range(k):
            rng = (i*np.pi*2/k, (i + 1)*np.pi*2/k)
            if (any(r > np.pi/2 and r < 3*np.pi/2 for r in rng)  # one of the bounds is on this range
                    or rng[0] <= np.pi/2 and rng[1] >= 3*np.pi/3):  # bounds encompass this range
                # can see x0 wall
                x = max(min_dist, pos[0] - xbound[0])
                if rng[0] <= np.pi and np.pi <= rng[1]:
                    ang = 0  # np.pi-np.pi
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, np.pi))) - np.pi
                inv_d = np.cos(ang)/x
                k_tant[i] = max(inv_d, k_tant[i])

            if rng[0] < np.pi/2 or rng[1] > 3*np.pi/2:
                # can see x1 wall
                x = max(min_dist, xbound[1] - pos[0])
                if i == 0 or i == k - 1:
                    ang = 0
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, 0)))
                # d=x/np.cos(ang)
                inv_d = np.cos(ang)/x
                k_tant[i] = max(inv_d, k_tant[i])
            if rng[1] > np.pi:
                # can see y0 wall
                y = max(min_dist, pos[1] - ybound[0])
                if rng[0] <= np.pi*3/2 and np.pi*3/2 <= rng[1]:
                    ang = 0  # np.pi*3/2-np.pi*3/2
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, np.pi*3/2))) - np.pi*3/2

                inv_d = np.cos(ang)/y
                k_tant[i] = max(inv_d, k_tant[i])
            if rng[0] < np.pi:
                # can see y1 wall
                y = max(min_dist, ybound[1] - pos[1])

                if rng[0] <= np.pi/2 and np.pi/2 <= rng[1]:
                    ang = 0  # np.pi/2-np.pi/2
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, np.pi/2))) - np.pi/2

                inv_d = np.cos(ang)/y
                k_tant[i] = max(inv_d, k_tant[i])
        return k_tant


class FSMk_tant_obstacle_surround(FSMxyBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 height_range,
                 obstacles,
                 obstacle_paths,
                 obstacle_zone,
                 activation_range,
                 end_time,
                 k=8,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100
                 ):
        """
        blimp sees the closest neighbor on each octant as well as obstacle vectors,
            rewarded for surrounding an obstacle

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param soln: neural network function call for blimp to act
        @param height_range: R^2, height range to keep blimps at
        @param obstacles: number of obstacles to randomly spawn in
        @param obstacle_paths: paths to obstacles to spawn in, list chosen from uniformly
        @param obstacle_zone: (RxR U R)^3 the spawn area of obstacles
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param activation_range: range where blimps are registered as 'surrounding' an obstacle
            also will keep obstacles at least double this distance apart so no overlap
        @param end_time: time it takes for experiment to end
        @param k: number of divisions to split sensing
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
            soln=soln,
            height_range=height_range,
            use_ultra=False,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.end_time = end_time
        self.k = k
        self.obstacles = obstacles
        self.obstacle_zone = obstacle_zone
        self.obstacle_paths = obstacle_paths
        self.activation_range = activation_range
        self.obs_handles = None

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        self.obs_handles = []
        self.obs_poses = []
        for _ in range(self.obstacles):
            obs = self.obstacle_paths[np.random.randint(0, len(self.obstacle_paths))]
            hand = self.spawnModel(obs,
                                   lambda: (0, 0, 0),
                                   spawn_tries=1,
                                   orient_rng=lambda: (0, 0, (0, 2*np.pi))
                                   )
            obs_pos = None
            for i in range(self.spawn_tries):
                self.moveObject(handle=hand, pos_rng=lambda: self.obstacle_zone,
                                orient_rng=lambda: (0, 0, (0, 2*np.pi)))
                obs_pos = self.get_object_pos(hand)[:2]
                too_close = False
                for p2 in self.obs_poses:
                    if np.linalg.norm(obs_pos - p2) < self.activation_range*2:
                        too_close = True
                        break
                if not too_close:
                    break
            self.obs_poses.append(obs_pos)
            self.obs_handles.append(hand)
            self.add_problem_model(hand)
        super().spawnThings()

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^(k+2*num_obstacles) np array
        """

        k_tant = self.global_get_inv_dist_2d_k_tant(agent_id, is_neigh=lambda id0, id1: True, k=self.k, spin=True)
        pos = self.get_position(agent_id=agent_id, use_ultra=False, spin=False)[:2]
        return np.concatenate([k_tant] + [obs_pos - pos for obs_pos in self.obs_poses])

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: the largest angle arc uncovered by a blimp near an object
            if there are too few blimps near an object, there is a penalty of the distance to the closest blimp
            if multiple objects, this value is summed
        """
        obj = 0
        for obs_pos in self.obs_poses:
            angles = []
            nearest_dist = float('inf')
            for agent_id in self.agentData:
                xy = self.get_position(agent_id, use_ultra=False)[:2]
                vec = xy - obs_pos
                nearest_dist = min(nearest_dist, np.linalg.norm(vec))
                if np.linalg.norm(vec) < self.activation_range:
                    angle = np.arctan2(vec[1], vec[0])
                    angles.append(angle%(2*np.pi))
                bug = self.get_state(agent_id)["DEBUG"]
                if bug == 0.:
                    return None
            angles.sort()
            if len(angles) < 2:
                max_dist = 2*np.pi
                obj -= nearest_dist  # penalty
            else:
                max_dist = (angles[0] + 2*np.pi) - angles[-1]
                for i in range(len(angles) - 1):
                    max_dist = max(max_dist, angles[i + 1] - angles[i])
            obj -= max_dist

        return obj

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time


class FSMxyzBlimp(blimpFSM):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        each blimp returns an xyz vector to go to

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param soln: neural network function call for blimp to act
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
            soln=soln,
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


class FSMl_k_tant_area_coverage(FSMxyzBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 bounds,
                 obstacles,
                 obstacle_paths,
                 use_ultra,
                 end_time,
                 l=3,
                 k=4,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        each blimp senses the distance on each l-k-tant (defaults to 3,4)
            rewarded for closeness to randomly sampled points

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param soln: neural network function call for blimp to act
        @param bounds: (RxR)^3, x bounds and y bounds to test for the area covered
            the goal function will uniformly choose some points in this area and rate the blimps based on closeness
        @param obstacles: number of obstacles to randomly spawn in
        @param obstacle_paths: paths to obstacles to spawn in, list chosen from uniformly
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param end_time: time it takes for experiment to end
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
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
            soln=soln,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.end_time = end_time
        self.bounds = bounds
        self.use_ultra = use_ultra
        self.dimension_split = int(num_agents**(1/3))
        self.l = l
        self.k = k
        self.obstacles = obstacles
        self.obstacle_paths = obstacle_paths
        self.obs_handles = None

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        self.obs_handles = []
        for _ in range(self.obstacles):
            obs = self.obstacle_paths[np.random.randint(0, len(self.obstacle_paths))]
            hand = self.spawnModel(obs,
                                   lambda: self.sample_from_bounds(1).flatten(),
                                   spawn_tries=1,
                                   orient_rng=lambda: ((0, 2*np.pi), (0, 2*np.pi), (0, 2*np.pi))
                                   )
            self.obs_handles.append(hand)
            self.add_problem_model(hand)
        super().spawnThings()

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """
        l_k_tant = self.global_get_inv_dist_3d_l_k_tant(agent_id, is_neigh=lambda id0, id1: True, l=self.l, k=self.k,
                                                        min_dist=.01, spin=True)
        return l_k_tant.reshape((-1, 1))

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
        boxes = np.array([[[0 for _ in range(self.dimension_split)]
                           for _ in range(self.dimension_split)]
                          for _ in range(self.dimension_split)])
        xbound, ybound, zbound = self.bounds
        for agent_id in self.agentData:
            xyz = self.get_position(agent_id, use_ultra=False)
            x, y, z = xyz
            xbox = self.dimension_split*(x - xbound[0])/(xbound[1] - xbound[0])
            ybox = self.dimension_split*(y - ybound[0])/(ybound[1] - ybound[0])
            zbox = self.dimension_split*(z - zbound[0])/(zbound[1] - zbound[0])

            xbox = np.clip(xbox, 0, self.dimension_split - .5)
            ybox = np.clip(ybox, 0, self.dimension_split - .5)
            zbox = np.clip(zbox, 0, self.dimension_split - .5)
            boxes[int(xbox), int(ybox), int(zbox)] += 1

            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None

        for x in boxes:
            for xy in x:
                for xyz in xy:
                    p = xyz/self.num_agents
                    if p > 0:
                        entropy += p*np.log(1/p)
        return entropy

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment
        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def sample_from_bounds(self, n):
        """
        samples uniformly from self.bounds
        @param n: number of samples to take
        @return: R^3 random sample
        """
        return np.random.uniform([b[0] for b in self.bounds], [b[1] for b in self.bounds], (n, len(self.bounds)))


class FSMl_k_tant_wall_sense_area_coverage(FSMl_k_tant_area_coverage):

    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
                 bounds,
                 obstacles,
                 obstacle_paths,
                 use_ultra,
                 end_time,
                 l=3,
                 k=4,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        each blimp senses the distance on each l-k-tant (defaults to 3,4)
            rewarded for closeness to randomly sampled points

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param soln: neural network function call for blimp to act
        @param bounds: (RxR)^3, x bounds and y bounds to test for the area covered
            the goal function will uniformly choose some points in this area and rate the blimps based on closeness
        @param obstacles: number of obstacles to randomly spawn in
        @param obstacle_paths: paths to obstacles to spawn in, list chosen from uniformly
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param end_time: time it takes for experiment to end
        @param l: divisions of phi to consider when using spherical coordinates
        @param k: divisions of theta to consider when using spherical coordinates
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
                         blimpPath=blimpPath,
                         soln=soln,
                         bounds=bounds,
                         obstacles=obstacles,
                         obstacle_paths=obstacle_paths,
                         use_ultra=use_ultra,
                         end_time=end_time,
                         l=l,
                         k=k,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries
                         )

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^k np array
        """

        neighbor_l_k_tant = self.global_get_inv_dist_3d_l_k_tant(agent_id, is_neigh=lambda id0, id1: True, k=self.k,
                                                                 l=self.l,
                                                                 spin=True)
        bounds_l_k_tant = self.get_bounds_l_k_tant(self.get_position(agent_id, spin=False), k=self.k, l=self.l)
        l_k_tant = np.max((neighbor_l_k_tant, bounds_l_k_tant), axis=0)
        return l_k_tant.reshape((-1, 1))

    def get_bounds_l_k_tant(self, pos, l, k, min_dist=.01):
        """
        gets the inverse distance to nearest bound for each k-tant

        @param pos: position to start from
        @param l: l-tants
        @param k: k-tants
        @return R^k array with inverse distance to closest bound on each k-tant
        """
        k_tant = np.zeros(k)
        xbound, ybound = self.bounds
        for i in range(k):
            rng = (i*np.pi*2/k, (i + 1)*np.pi*2/k)
            if (any(r > np.pi/2 and r < 3*np.pi/2 for r in rng)  # one of the bounds is on this range
                    or rng[0] <= np.pi/2 and rng[1] >= 3*np.pi/3):  # bounds encompass this range
                # can see x0 wall
                x = max(min_dist, pos[0] - xbound[0])
                if rng[0] <= np.pi and np.pi <= rng[1]:
                    ang = 0  # np.pi-np.pi
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, np.pi))) - np.pi
                inv_d = np.cos(ang)/x
                k_tant[i] = max(inv_d, k_tant[i])

            if rng[0] < np.pi/2 or rng[1] > 3*np.pi/2:
                # can see x1 wall
                x = max(min_dist, xbound[1] - pos[0])
                if i == 0 or i == k - 1:
                    ang = 0
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, 0)))
                # d=x/np.cos(ang)
                inv_d = np.cos(ang)/x
                k_tant[i] = max(inv_d, k_tant[i])
            if rng[1] > np.pi:
                # can see y0 wall
                y = max(min_dist, pos[1] - ybound[0])
                if rng[0] <= np.pi*3/2 and np.pi*3/2 <= rng[1]:
                    ang = 0  # np.pi*3/2-np.pi*3/2
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, np.pi*3/2))) - np.pi*3/2

                inv_d = np.cos(ang)/y
                k_tant[i] = max(inv_d, k_tant[i])
            if rng[0] < np.pi:
                # can see y1 wall
                y = max(min_dist, ybound[1] - pos[1])

                if rng[0] <= np.pi/2 and np.pi/2 <= rng[1]:
                    ang = 0  # np.pi/2-np.pi/2
                else:
                    ang = min(rng, key=lambda a: abs(self.angle_diff(a, np.pi/2))) - np.pi/2

                inv_d = np.cos(ang)/y
                k_tant[i] = max(inv_d, k_tant[i])
        return k_tant


class FSMxyz_zero_Blimp(FSMxyzBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
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
        @param soln: neural network function call for blimp to act
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
            soln=soln,
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
                return None
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment
        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time


class FSMl_k_tant_clump_blimp(blimpFSM):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 soln,
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
            soln=soln,
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
        l_k_tant = self.get_neighbors_3d_l_k_tant(agent_id=agent_id,
                                                  is_neigh=lambda id1, id2: self.within_range(id0=id1,
                                                                                              id1=id2,
                                                                                              rng=self.rng,
                                                                                              spin=False), l=self.l,
                                                  k=self.k, spin=True)
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
            bug = self.get_state(id1)["DEBUG"]
            if bug == 0.:
                return None
        return np.mean(s)

    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time
