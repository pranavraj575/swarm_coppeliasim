import sys, csv

from src.swarm_expiriment import *
from collections import defaultdict


class chainSiphon(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 goal_value,
                 start_zone=lambda i: ((.5, 8), (-3, 3), (1, 1.25)),
                 end_time=300,
                 scenePath=wall_climb_path,
                 blimpPath=narrow_blimp_path,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=.1,
                 goal_field=None,
                 weight=None,
                 point_obstacles=(),
                 obs_vector=lambda pos: np.array((0, 0, 0)),
                 max_speed=0.75,
                 obs_range=1.5,
                 obs_rep=10,
                 etta=0.2,
                 neighbor_range=2.25,
                 max_ljp=500,
                 workspace=((-5, 6), (-2, 2), (1, 1.25)),
                 goal_test=lambda p: False,
                 use_ultra=True,
                 ):
        super().__init__(num_agents,
                         start_zone,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         sim=sim,
                         simId=simId,
                         wakeup=wakeup,
                         sleeptime=sleeptime)

        self.weight = weight
        self.end_time = end_time
        self.point_obstacles = point_obstacles
        self.obs_vector = obs_vector
        self.max_speed = max_speed
        self.obs_range = obs_range
        self.obs_rep = obs_rep
        self.etta = etta
        self.neighbor_range = neighbor_range
        self.max_ljp = max_ljp
        self.workspace = workspace
        self.goal_test = goal_test
        self.use_ultra = use_ultra
        self.set_goal_value(goal_value=goal_value, goal_field=goal_field)
        self.swarm_data = defaultdict(lambda: np.array([0.]*8))

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

    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)
        @return: boolean, whether or not experiment is done
        """
        # self.spin()
        height = False
        for agent_id in self.agentData.keys():
            self.spin([agent_id])
            pos = self.get_position(agent_id, use_ultra=self.use_ultra, spin=False)

            real_pos = self.get_position(agent_id, use_ultra=False, spin=False)
            # for real x,y,z

            #####################################################################################################
            # Goal (normalized)

            v_goal = self.safe_norm(self.goal_field(real_pos))

            #####################################################################################################
            # Obstacles
            v_obs_tot = np.zeros(3)
            v_obs_tot += self.point_obs_force(real_pos, self.point_obstacles, self.obs_range, self.obs_rep)
            v_obs_tot += self.obs_vector(real_pos)

            #####################################################################################################
            # LJP

            neigh_min_queue_pos = self.num_agents
            min_dist = self.neighbor_range
            min_dist_element = None
            v_visc = np.zeros(3)
            neighbors = []
            for key in self.swarm_data:
                item = self.swarm_data[key]
                x = item[0]
                if x != agent_id:
                    dPos = item[1:4] - real_pos
                    dist = np.linalg.norm(dPos)
                    if dist <= 0:
                        dist = .00001
                    if dist < self.neighbor_range:
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

            if self.goal_test(real_pos):
                queue_value = 1

            if neighbors:
                min_size = self.num_agents + 1
                for item in neighbors:
                    dPos = item[1:4] - real_pos

                    dist = np.linalg.norm(dPos)
                    if dist == 0:
                        dist = 0.00001

                    dPos = self.safe_norm(dPos)

                    ljp = dPos*(24*self.etta*(
                            -26/((0.6*(dist + 0.75))**14) + 7/((0.6*(dist + 0.75))**8)))

                    if item[4] < queue_value:
                        queue_value = item[4] + 1

                    if queue_value > self.num_agents:
                        queue_value = self.num_agents

                    if item is min_dist_element and neigh_min_queue_pos < queue_value:
                        v_min = dPos

                    v_visc += ljp

                    ## NEW doesnt this work?
                    visited = {agent_id}
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
            if np.linalg.norm(v_visc) > self.max_ljp:
                v_visc = self.safe_norm(v_visc)*self.max_ljp
            #####################################################################################################
            # Workspace
            v_bound = np.zeros(3)
            # USE ULTRASOUND POS HERE, matches the 'climbing' behavior
            for k in range(3):
                temp = np.zeros(3)
                if pos[k] < self.workspace[k][0]:
                    temp[k] += self.workspace[k][0] - pos[k]  # 1.
                if pos[k] > self.workspace[k][1]:
                    temp[k] += self.workspace[k][1] - pos[k]  # -1.
                v_bound += temp

            #####################################################################################################
            # Full

            v_full = v_goal*self.weight[0] + \
                     v_obs_tot*self.weight[1] + \
                     v_visc*self.weight[2] + \
                     new_v_min*self.weight[3] + \
                     v_bound*self.weight[4]
            if height:
                v_full[2] = (height - pos[2])*.1
                # USE ULTRASOUND POS HERE
            if np.linalg.norm(v_full) > self.max_speed:
                v_full = self.safe_norm(v_full)*self.max_speed

            self.move_agent(agent_id, v_full)
            self.data_publish(agent_id,
                              (agent_id,
                               real_pos[0],
                               real_pos[1],
                               real_pos[2],
                               queue_value,
                               best_leader,
                               obj_value,
                               len(neighbors)
                               ))
        return self.sim.getSimulationTime() > self.end_time

    def data_publish(self, agent_id, agent_data):
        self.swarm_data[agent_id] = np.array(agent_data)

    def goal_data(self):
        succ = 0
        for agent_id in self.agentData:
            if self.get_state(agent_id)['x'] < 0:
                succ += 1

            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None
        return succ


specifier = ''
trials = 30
for t in range(trials):
    print()
    print('trial:', t)
    agent_range = (1, 31)
    for mode in ('control',
                 'chain',
                 'leader',
                 'LJP'):
        print()
        print('mode:', mode)

        if mode == 'control':
            # (goal, obstacle, viscosity, min, bounding)
            weights = (.75, .0, .0, 0., .75,)
        elif mode == 'chain':
            weights = (.75, .0, .002, 1., .75,)
        elif mode == 'leader':
            weights = (.75, .0, .0, 1., .75,)
        elif mode == 'LJP':
            weights = (.75, .0, .002, 0., .75,)
        else:
            raise Exception("run with --control, --chain, --leader, or --LJP")

        save_dir = os.path.join(DIR, 'chain_siphon', 'output', mode)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        else:
            print("WARNING: continuing saved output                   ")
        fields = ['agents', 'successful']
        for agents in range(agent_range[0], agent_range[1]):
            filename = os.path.join(save_dir, 'data' + specifier + '.csv')

            oldfields = None
            olddata = []

            reccy = defaultdict(lambda: 0)
            # number of times weve tried each agent

            if os.path.exists(filename):
                csvfile = open(filename)
                spamreader = csv.reader(csvfile)
                for row in spamreader:
                    if oldfields is None:
                        oldfields = row
                    else:
                        olddata.append([float(r) for r in row])
                        reccy[int(float(row[0]))] += 1
                csvfile.close()
            skipping = reccy[agents] > t

            print('MODE:', mode, ';\tAGENTS:', agents, ':\t SKIPPING' if skipping else '',
                  end='\r' if skipping else '\n')
            if skipping:
                continue
            passed = None
            while passed is None:
                bb = chainSiphon(agents,
                                 goal_value=lambda pos: pos[0],
                                 goal_field=lambda pos: np.array((-1., 0., 0.)),
                                 weight=weights,
                                 wakeup=[COPPELIA_WAKEUP + ' -h'])
                passed = bb.experiments(1)
                bb.kill()
                if passed is None:
                    print("FAILED, TRYING AGAIN")
                    time.sleep(1)

            passed = passed[0]

            newrow = [agents, passed]

            olddata.append(newrow)
            # olddata.sort(key=lambda row:int(row[0]))
            # increasing by num agents

            csvfile = open(filename, 'w')
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(fields)
            csvwriter.writerows(olddata)
            csvfile.close()
