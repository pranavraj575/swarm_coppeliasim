from src.swarm_expiriment import *
from collections import defaultdict
import torch, copy, lkh
from sklearn.cluster import KMeans
from pathlib import Path

SOLVER_PATH = str(Path('~/LKH-3.0.6/LKH').expanduser())


class travelingSalesBlimp(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 num_points,
                 spawn_pt_info,
                 alpha,
                 depot,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=1,
                 spawn_tries=100,
                 scenePath=empty_path,
                 blimpPath=narrow_blimp_path,
                 goalPath=os.path.join(DIR, "models", "goal.ttm"),
                 speed=.5,
                 blimp_tol=.2,
                 depot_tol=1.,
                 debug=False,
                 ):
        """
        Class to run a TSP problem with blimps. The 'experiment' will just run a predetermined path for each blimp
            Important part is how the paths are calculated

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param num_points: number of points to spawn
        @param spawn_pt_info: N -> dict; given point index i, spawns dictionary with ith point's position and tau
                required keys: ['pos', 'tau']
        @param alpha: alpha parameter for time discounting entropy
        @param depot: unique POI that blimps must start at and return to
        @param lr: learning rate for ADAM gradient descent
        @param epochs: number of epochs to run grad descent
        @param grads_per_epoch: number of trials per epoch, float('inf') if just use converge_tol
        @param converge_tol: tolerance to converge grad descent, None if just use grads_per_epoch
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param goalPath: path to object to indicate POI
        @param speed: max speed to move the blimps at
        @param blimp_tol: space around target that blimps are registered as 'touching' target
        @param depot_tol: space around depot that blimps are registered as at depot
        @param debug: boolean, whether to print debug stuff
        """
        if not converge_tol and grads_per_epoch == float('inf'):
            raise Exception('this cant happen')
        super().__init__(num_agents=num_agents,
                         start_zone=start_zone,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries)
        self.num_points = num_points
        self.alpha = alpha
        self.lr = lr
        self.epochs = epochs
        self.grads_per_epoch = grads_per_epoch
        self.converge_tol = converge_tol
        self.spawn_pt_info = spawn_pt_info
        self.goalPath = goalPath
        self.pointData = defaultdict(lambda: dict())
        self.speed = speed
        self.blimp_tol = blimp_tol
        self.visited = []
        self.waiting = defaultdict(lambda: dict())
        self.goal_list = None
        self.depot = depot
        self.depot_tol = depot_tol
        self.agentGoals = None
        self.dwell_solution = dict()
        self.non_depot_handles = None
        self.times_created = False
        self.debug = debug

    ####################################################################################################################
    # init/shutdown functions (needs implementation in subclass)
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        super().spawnThings()
        self.agentGoals = {id: 0 for id in self.agentData}

        self.pointData = defaultdict(lambda: dict())

        depot_handle = self.spawnModel(self.goalPath, lambda: self.depot, 1)
        self.pointData[depot_handle]['pos'] = self.depot
        self.pointData[depot_handle]['edge_cost'] = dict()
        self.pointData[depot_handle]['order_added'] = -1
        self.pointData[depot_handle]['is depot'] = True
        self.pointData[depot_handle]['handle'] = depot_handle
        self.set_color(depot_handle, [255, 0, 255])
        self.depotData = self.pointData[depot_handle]
        self.non_depot_handles = []
        for i in range(self.num_points):
            pt_info = self.spawn_pt_info(i)
            pt_handle = self.spawnModel(self.goalPath, lambda: pt_info['pos'], spawn_tries=1)
            self.non_depot_handles.append(pt_handle)
            # self.points.append((pt, goalHandle))
            self.pointData[pt_handle]['pos'] = pt_info['pos']
            self.pointData[pt_handle]['tau'] = pt_info['tau']
            self.pointData[pt_handle]['order_added'] = i
            self.pointData[pt_handle]['edge_cost'] = dict()
            self.pointData[pt_handle]['is depot'] = False
            self.pointData[pt_handle]['handle'] = pt_handle
            self.set_color(pt_handle, [255, 0, 0])
        self.visited = []
        self.waiting = defaultdict(lambda: dict())
        # self.goal_list = self.make_goal_list()
        # self.create_times()

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def find_handle(self, pt):
        """
        find a POI by its location (approx)

        @param pt: R^3 point to find handle of
        @return: handle of POI
        """
        return min(self.pointData, key=lambda key: np.linalg.norm(self.pointData[key]['pos'] - pt))

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        if self.goal_list is None:
            self.sim.pauseSimulation()
            self.goal_list = self.make_goal_list()
            self.create_times()
            self.sim.startSimulation()
        done = True
        for agent_id in self.agentData:
            pos = self.get_position(agent_id=agent_id, use_ultra=False, spin=True)
            goals = self.goal_list[agent_id]
            goal_index = self.agentGoals[agent_id]
            if goal_index >= len(goals):
                g_hand = None
                goal = None
            else:
                done = False
                g_hand = goals[goal_index]
                goal = self.pointData[g_hand]['pos']
                is_depot = self.pointData[g_hand]['is depot']
                if np.linalg.norm(goal - pos) <= (self.depot_tol if is_depot else self.blimp_tol):
                    # reached
                    if g_hand in self.waiting[agent_id]:
                        spent = self.sim.getSimulationTime() - self.waiting[agent_id][g_hand]
                        if spent >= self.pointData[g_hand]['time']:
                            self.waiting[agent_id].pop(g_hand)
                            self.visited.append(g_hand)
                            self.agentGoals[agent_id] += 1
                            if not is_depot:
                                self.set_color(g_hand, [0, 255, 0])
                    else:
                        self.waiting[agent_id][g_hand] = self.sim.getSimulationTime()
                else:
                    if g_hand in self.waiting:
                        self.waiting[agent_id].pop(g_hand)

            if goal is None:
                vec = np.zeros(3)
            else:
                vec = (goal - pos)*.5
                if np.linalg.norm(vec) > self.speed:
                    vec = self.speed*vec/np.linalg.norm(vec)
            self.move_agent(agent_id, vec)
        return done

    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: 0, this is not important here
        """
        return 0

    ####################################################################################################################
    # Cost functions
    ####################################################################################################################
    def cost_fn(self, h1, h2):
        """
        returns cost (euclidean distance) between two POIs

        @param h1: handle 1
        @param h2: handle 2
        @return: euclidean distance
        """
        return np.linalg.norm(self.pointData[h1]['pos'] - self.pointData[h2]['pos'])

    def get_cost(self, h1, h2, use_dict=True):
        """
        returns cost (defined in cost_fn) between two POIs

        @param h1: handle 1
        @param h2: handle 2
        @param use_dict: whether to calculate every time or use lookup table
        @return: cost
        """
        if use_dict:
            if h2 not in self.pointData[h1]['edge_cost']:
                cost = self.cost_fn(h1, h2)
                self.pointData[h1]['edge_cost'][h2] = cost
            return self.pointData[h1]['edge_cost'][h2]
        else:
            return self.cost_fn(h1, h2)

    def get_path_cost(self, path):
        """
        returns total path cost of a path around POIS

        @param path: list of POI handles
        @return: path cost (using costs defined in cost_fn)
        """
        S = 0
        for i in range(len(path) - 1):
            h1 = path[i]
            h2 = path[i + 1]
            S += self.get_cost(h1, h2)
        return S

    ####################################################################################################################
    # entropy functions
    ####################################################################################################################
    def I(self, D, taus):
        """
        Information gain of a vector of POIs with tau value 'taus', if dwell times are 'D'

        @param D: M x 1 torch tensor of dwell times
        @param taus: M x 1 torch tensor of tau values
        @return: M x 1 torch tensor of entropy gain values for each POI
        """
        E = .5*torch.exp(-D/taus)
        eyes = (1 - E)*torch.log(1 - E) - E*(np.log(2) + D/taus) + np.log(2)
        return eyes

    def prop_to_J(self, D, taus):
        """
        Returns value proportional to the discounted entropy gain with dwell times 'D', tau values 'taus'
            must be multiplied by exp(alpha * path cost for partition) to get actual discounted entropy

        @param D: M x 1 torch tensor of dwell times
        @param taus:  M x 1 torch tensor of tau values
        @return: torch constant of value proportional to discounted entropy gain (without beta)
        """
        eyes = self.I(D, taus)
        return torch.exp(-self.alpha*torch.sum(D))*torch.sum(eyes)

    def create_times(self, goal_list=None):
        """
        uses torch gradient descent to find optimal dwell times for a given partition/TSP solution

        @param goal_list: list of goals for each agent to visit, if None we will make one using default method
        """
        self.times_created = True
        if goal_list is None:
            if self.goal_list is None:
                print("WARNING: NO GOAL LIST, using default method")
                self.goal_list = self.make_goal_list()
            goal_list = self.goal_list
        self.pointData[self.depotData['handle']]['time'] = 0.  # depot point
        self.dwell_solution = defaultdict(lambda: [])
        for handles in goal_list:
            H = [h for h in handles if not self.pointData[h]['is depot']]
            taus = torch.tensor([[self.pointData[hand]['tau']] for hand in H])
            D = torch.rand((taus.size()), requires_grad=True)
            for _ in range(self.epochs):
                optim = torch.optim.Adam(params=[D], lr=self.lr)
                i = 0
                old_val = float('inf')
                while i < self.grads_per_epoch:
                    loss = -self.prop_to_J(D, taus)
                    # negative since maximization
                    optim.zero_grad()
                    loss.backward()
                    optim.step()
                    if any(D < 0):
                        D = copy.copy(D.clamp(0.))
                        optim = torch.optim.Adam(params=[D], lr=self.lr)
                    i += 1
                    if self.converge_tol and abs(old_val - loss.item()) < self.converge_tol:
                        break
                    old_val = loss.item()
            for i, hand in enumerate(H):
                self.pointData[hand]['time'] = D[i]
            self.dwell_solution['handles'].append(H)
            self.dwell_solution['taus'].append(taus.squeeze().tolist())
            self.dwell_solution['times'].append(D.squeeze().tolist())

    def get_tot_disc_entropy(self, goal_list=None):
        """
        gets total discounted entropy of a given goal list

        @param goal_list: list of list of handles, length num_agents, total of num_points elements that are not the depot
            (partition/TSP solution)
            if None, we will use default method
        @return: discounted entropy
        """
        if goal_list is None:
            if self.goal_list is None:
                self.goal_list = self.make_goal_list()
            goal_list = self.goal_list
        S = 0
        for handles in goal_list:
            H = [h for h in handles if not self.pointData[h]['is depot']]
            pc = self.get_path_cost(handles)
            D = torch.tensor([[self.pointData[hand]['time']] for hand in H])
            taus = torch.tensor([[self.pointData[hand]['tau']] for hand in H])
            J = np.exp(-self.alpha*pc)*self.prop_to_J(D, taus)
            S += J
        return S

    def get_disc_entropy_from_part(self, partition):
        """
        gets the optimal discounted entropy from a partition

        @param partition: list of lists, length num_agents, total of num_points elements
            partition of the non-depot POIs
        @return: cost of solution, after running lkh and gradient descent
        """
        goal_list = self.make_goal_list(partition)
        self.create_times(goal_list)
        cost = self.get_tot_disc_entropy(goal_list).item()
        return cost

    ####################################################################################################################
    # TSP/partition functions
    ####################################################################################################################
    def make_goal_list(self, partition=None):
        """
        gives each blimp a list of points to go to, in order

        @param partition: list of lists, length num_agents, total of num_points elements
            partition of the non-depot POIs
            if None, will use default method
        @return: list of lists, length num_agents, total number of non depot elements must be num_points
                elements are keys of points for each agent to visit
        """
        if partition is None:
            partition = self.make_goal_partition()
        out = []
        for part in partition:
            cycle = part + [self.depotData['handle']]  # adds depot to partition
            cycle = self.lkhSolve(cycle)
            i = cycle.index(self.depotData['handle'])
            cycle = [cycle[(j + i)%len(cycle)] for j in range(len(cycle))]
            cycle.append(self.depotData['handle'])
            out.append(cycle)
        return out

    def lkhSolve(self, handles, factor=1000):
        """
        given handles, gives approximation of optimal order
            https://pypi.org/project/lkh/
            http://webhotel4.ruc.dk/~keld/research/LKH-3/LKH-3_REPORT.pdf

        @param handles: list of handles to order
        @param factor: lkh takes integers, so we multiply coordinates by factor to increase precision
        @return: list of handles in order
        """
        if len(handles) < 3:
            return handles
        fn = 'temp' + str(time.time()).replace('.', '_') + '.txt'
        fn = os.path.join(DIR, fn)
        f = open(fn, 'w')
        f.write('TYPE : TSP\n')
        f.write('DIMENSION : ' + str(len(handles)) + '\n')
        f.write('EDGE_WEIGHT_TYPE : EUC_3D\n')
        f.write('NODE_COORD_SECTION\n')
        node_to_key = dict()
        for i, key in enumerate(handles):
            f.write(str(i + 1) + ' ')
            for v in self.pointData[key]['pos']:
                f.write(str(int(round(v*factor))) + ' ')
            f.write('\n')
            node_to_key[i + 1] = key
        f.close()
        lis = lkh.solve(SOLVER_PATH, problem_file=fn, max_trials=10000, runs=10)
        os.remove(fn)
        return [node_to_key[i] for i in lis[0]]

    def make_goal_partition(self):
        return self.make_kmeans_partition()

    ####################################################################################################################
    # K-means partitioning
    ####################################################################################################################
    def make_kmeans_partition(self):
        """
        default partition solver, currently used KMeans
            partitions the goals into a list for each blimp (order does not matter)

        @return: list of lists, length num_agents, total number of elements must be num_points (ignores depot)
                elements are handles of POIs for each agent to visit
        """
        H = self.non_depot_handles
        kmeans = KMeans(n_clusters=self.num_agents, n_init=10).fit(np.array([self.pointData[h]['pos'] for h in H]))
        output = [[] for _ in range(self.num_agents)]

        for i, lab in enumerate(kmeans.labels_):
            output[lab].append(H[i])
        return output

    ####################################################################################################################
    # Spectral partitioning
    ####################################################################################################################
    def make_laplacian(self):
        """
        returns graph laplacian of the non-depot POIs

        edge weights are negative distance (plus some constant to make all of them positive)
            This is negated since higher connectivity should be correlated with smaller distance
        """
        H = self.non_depot_handles
        n = len(H)
        A = np.zeros((n, n))
        max_dist = 0
        for i in range(n):
            for j in range(i + 1, n):
                c = self.get_cost(H[i], H[j])
                c = np.exp(-c)
                # since we are minimizing the sum of costs, larger connectivity should be correlated with less cost
                A[i][j] = c
                A[j][i] = c
                max_dist = max(c, max_dist)
        # A=max_dist-A
        # since we are minimizing the sum of costs, larger connectivity should be correlated with less cost
        row_sums = np.sum(A, axis=1)
        D = np.zeros((n, n))
        for i in range(n):
            D[i][i] = row_sums[i]
        return D - A

    def make_spectral_partition(self):
        """
        @return: list of lists, length num_agents, total number of elements must be num_points (ignores depot)
                elements are handles of POIs for each agent to visit

        https://chrisyeh96.github.io/2021/03/06/k-way-spectral-clustering.html#k-way-clustering
        https://dl.acm.org/doi/10.5555/2980539.2980649
        """
        k = self.num_agents
        H = self.non_depot_handles
        if k > len(H):
            # if we outnumber the number of POIs, just send one blimp to each POI
            return [[h] for h in H] + [[] for _ in range(k - len(H))]
        L = self.make_laplacian()
        print(L)
        eigenvalues, eigenvectors = np.linalg.eig(L)
        F = eigenvectors[:, 0:k]  # the k smallest eigenvectors
        # now F[i,:] is the 'feature vector' of the ith POI
        for i in range(len(F)):
            F[i, :] = F[i, :]/np.linalg.norm(F[i, :])
            # the norm will never be 0 since the first eigenvector should be all 1s

        # now we will cluster the feature vector according to k-means
        kmeans = KMeans(n_clusters=self.num_agents, n_init=10).fit(np.array([F[i, :] for i in range(len(H))]))
        output = [[] for _ in range(self.num_agents)]
        for i, lab in enumerate(kmeans.labels_):
            output[lab].append(H[i])
        return output

    ####################################################################################################################
    # local search partitioning
    ####################################################################################################################

    def get_neighbors(self, partition):
        """
        returns all neighbors of partition attained by switching one element

        @param partition: partition of non-depot POI handles, list of lists
        """
        for i in range(len(partition)):
            for j in range(len(partition[i])):
                # flip element partition[i][j]
                ele = partition[i][j]
                pp = copy.deepcopy(partition)
                pp[i] = pp[i][:j] + pp[i][j + 1:]
                for k in range(len(partition)):
                    if k != i:
                        temp = copy.deepcopy(pp)
                        temp[k].append(ele)
                        yield temp

    def local_search_partitions(self, init_parts, checked=None):
        """
        local searches starting from init_parts as seeds
            keeps track of len(init_parts) local optima at each iteration

        @param init_parts: initial partitions to check
        @param checked: already ruled out partitions, defaults to nothing
        @return: list of lists, length num_agents, total number of elements must be num_points (ignores depot)
                elements are handles of POIs for each agent to visit
        """

        parts = init_parts
        objs = [self.get_disc_entropy_from_part(part) for part in parts]
        dones = [False for _ in range(len(init_parts))]
        if checked is None:
            checked = {self.string_from_part(part) for part in parts}
        while not all(dones):
            if self.debug:
                print('best partitions found:', parts)
                print('info gain:', objs)
                print('local optima:', dones)
            for i, part in enumerate(parts):
                if dones[i]:
                    continue
                best = None, None
                obj = objs[i]
                for neighbor in self.get_neighbors(part):
                    string = self.string_from_part(neighbor)
                    if string in checked: continue
                    checked.add(string)
                    ob = self.get_disc_entropy_from_part(neighbor)
                    if ob > obj and (best[1] is None or ob > best[1]):
                        best = (neighbor, ob)
                if best[0] is None:
                    dones[i] = True
                else:
                    parts[i], objs[i] = best
        best_part = parts[max(range(len(parts)), key=lambda i: objs[i])]
        return best_part

    def make_local_search_partition(self):
        """
        @return: list of lists, length num_agents, total number of elements must be num_points (ignores depot)
                elements are handles of POIs for each agent to visit
        """
        return self.local_search_partitions([self.make_kmeans_partition()])

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def string_from_part(self, part):
        """
        returns string representing partition (ith index is the partition that the ith handle is assigned to)

        @param partition: partition of non-depot POI handles, list of lists
        @return: string representation

        @note: inverse of part_from_string
        """
        s = []
        for h in self.non_depot_handles:
            for i, p in enumerate(part):
                if h in p:
                    s.append(i)
        return ''.join(str(ch) for ch in s)

    def part_from_string(self, string):
        """
        returns partition from its string representation

        @param string: string representation
        @return: partition of non-depot POI handles, list of lists

        @note: inverse of string_from_part
        """
        part = [[] for _ in range(self.num_agents)]
        for i, h in enumerate(self.non_depot_handles):
            part[int(string[i])].append(h)
        return part


class singleBlimp(travelingSalesBlimp):
    def __init__(self,
                 start_zone,
                 num_points,
                 spawn_pt_info,
                 alpha,
                 depot,
                 depot_tol=.1,
                 scenePath=empty_path,
                 blimpPath=narrow_blimp_path,
                 goalPath=os.path.join(DIR, "models", "goal.ttm"),
                 speed=.5,
                 blimp_tol=.2,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 msg_queue=10,
                 sleeptime=1,
                 spawn_tries=100, ):
        """
        Test, with single blimp
        """
        super().__init__(num_agents=1,
                         start_zone=start_zone,
                         num_points=num_points,
                         spawn_pt_info=spawn_pt_info,
                         alpha=alpha,
                         depot=depot,
                         depot_tol=depot_tol,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         goalPath=goalPath,
                         speed=speed,
                         blimp_tol=blimp_tol,
                         lr=lr,
                         epochs=epochs,
                         grads_per_epoch=grads_per_epoch,
                         converge_tol=converge_tol,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries
                         )


class doubleBlimp(travelingSalesBlimp):
    def __init__(self,
                 start_zone,
                 num_points,
                 spawn_pt_info,
                 alpha,
                 depot,
                 depot_tol=.1,
                 scenePath=empty_path,
                 blimpPath=narrow_blimp_path,
                 goalPath=os.path.join(DIR, "models", "goal.ttm"),
                 speed=.5,
                 blimp_tol=.2,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 msg_queue=10,
                 sleeptime=1,
                 spawn_tries=100, ):
        """
        Test, with two blimps, uses KMeans partition
        """
        super().__init__(num_agents=2,
                         start_zone=start_zone,
                         num_points=num_points,
                         spawn_pt_info=spawn_pt_info,
                         alpha=alpha,
                         depot=depot,
                         depot_tol=depot_tol,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         goalPath=goalPath,
                         speed=speed,
                         blimp_tol=blimp_tol,
                         lr=lr,
                         epochs=epochs,
                         grads_per_epoch=grads_per_epoch,
                         converge_tol=converge_tol,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries
                         )
