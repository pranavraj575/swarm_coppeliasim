import copy

from src.swarm_expiriment import *
from collections import defaultdict
import lkh
import time
import torch
from sklearn.cluster import KMeans

SOLVER_PATH = '/home/rajbhandari/LKH-3.0.6/LKH'


class travelingSalesBlimp(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 num_points,
                 spawn_pt_info,
                 alpha,
                 depot,
                 depot_tol=1.,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=1,
                 scenePath=empty_path,
                 blimpPath=narrow_blimp_path,
                 goalPath="/home/rajbhandari/projects/blimp_coppeliasim/models/goal.ttm",
                 speed=.5,
                 blimp_tol=.2
                 ):
        """
        @param num_points: number of points to spawn
        @param spawn_pt_info: N -> dict; given point index i, spawns dictionary with ith point's position and tau
                required keys: ['pos', 'tau']
        @param alpha: alpha parameter for time discounting entropy
        @param lr: learning rate for ADAM gradient descent
        @param epochs: number of epochs to run grad descent
        @param grads_per_epoch: number of trials per epoch, float('inf') if just use converge_tol
        @param converge_tol: tolerance to converge grad descent, None if just use grads_per_epoch
        @param goalPath: path to object to indicate POI
        @param speed: max speed to move the blimps at
        @param blimp_tol: space around target that blimps are registered as 'touching' target

        """
        if not converge_tol and grads_per_epoch == float('inf'):
            raise Exception('this cant happen')
        super().__init__(num_agents=num_agents,
                         start_zone=start_zone,
                         sim=sim,
                         simId=simId,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         scenePath=scenePath,
                         blimpPath=blimpPath)
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
        self.goal_list_made = False
        self.dwell_solution = dict()
        self.non_depot_handles = None
        self.times_created = False

    ####################################################################################################################
    # init/shutdown functions (needs implementation in subclass)
    ####################################################################################################################
    def spawnThings(self):
        super().spawnThings()
        self.agentGoals = {id: 0 for id in self.agentData}

        self.pointData = defaultdict(lambda: dict())

        depot_handle = self.spawnBlimp(self.goalPath, lambda: self.depot, 1)
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
            pt_handle = self.spawnBlimp(self.goalPath, lambda: pt_info['pos'], spawn_tries=1)
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
        #self.goal_list = self.make_goal_list()
        #self.create_times()

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def find_handle(self, pt):
        return min(self.pointData, key=lambda key: np.linalg.norm(self.pointData[key]['pos'] - pt))

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        for agent_id in self.agentData:
            pos = self.get_position(agent_id=agent_id, use_ultra=False, spin=True)
            goals = self.goal_list[agent_id]
            goal_index = self.agentGoals[agent_id]
            if goal_index >= len(goals):
                g_hand = None
                goal = None
            else:
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
                vec = (goal - pos) * .5
                if np.linalg.norm(vec) > self.speed:
                    vec = self.speed * vec / np.linalg.norm(vec)
            self.move_agent(agent_id, vec)

    def is_done(self):
        return self.goal_list_made and all(
            self.agentGoals[agent_id] >= len(self.goal_list[agent_id]) for agent_id in self.agentData)

    def goal_data(self):
        return None

    ####################################################################################################################
    # TSP/entropy functions
    ####################################################################################################################
    def I(self, D, taus):
        E = .5 * torch.exp(-D / taus)
        eyes = (1 - E) * torch.log(1 - E) - E * (np.log(2) + D / taus) + np.log(2)
        return eyes

    def prop_to_J(self, D, taus):
        eyes = self.I(D, taus)
        return torch.exp(-self.alpha * torch.sum(D)) * torch.sum(eyes)

    def create_times(self, goal_list=None):
        # assumes goal list is already made partitioning agents
        # thus, run grad descent to find optimal dwell times
        self.times_created=True
        if goal_list is None:
            if not self.goal_list_made:
                print("WARNING: NO GOAL LIST, using default method")
                self.make_goal_list()
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

    def _cost_fn(self, h1, h2):
        return np.linalg.norm(self.pointData[h1]['pos'] - self.pointData[h2]['pos'])

    def get_cost(self, h1, h2):
        if h2 not in self.pointData[h1]['edge_cost']:
            cost = self._cost_fn(h1, h2)
            self.pointData[h1]['edge_cost'][h2] = cost
        return self.pointData[h1]['edge_cost'][h2]

    def get_path_cost(self, path):
        # gets path cost, path is a list of handles
        S = 0
        for i in range(len(path) - 1):
            h1 = path[i]
            h2 = path[i + 1]
            S += self.get_cost(h1, h2)
        return S

    def get_tot_disc_entropy(self, goal_list=None):
        # total cost of partition/path, assumes make_goal_list and times was called
        if goal_list is None:
            goal_list = self.goal_list
        S = 0
        for handles in goal_list:
            H = [h for h in handles if not self.pointData[h]['is depot']]
            pc = self.get_path_cost(handles)
            D = torch.tensor([[self.pointData[hand]['time']] for hand in H])
            taus = torch.tensor([[self.pointData[hand]['tau']] for hand in H])
            J = np.exp(-self.alpha * pc) * self.prop_to_J(D, taus)
            S += J
        return S

    def make_goal_list(self, partition=None):
        """
        gives each blimp a list of points to go to, in order
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
            cycle = [cycle[(j + i) % len(cycle)] for j in range(len(cycle))]
            cycle.append(self.depotData['handle'])
            out.append(cycle)
        self.goal_list_made = True
        return out

    def lkhSolve(self, handles, factor=1000):
        """
        https://pypi.org/project/lkh/
        http://webhotel4.ruc.dk/~keld/research/LKH-3/LKH-3_REPORT.pdf
        given handles, gives optimal order
        @param handles: list of handles to order
        @param factor: lkh takes integers, so we multiply coordinates by factor to increase precision
        @return: list of handles in order
        """
        if len(handles) < 3:
            return handles
        fn = 'temp' + str(time.time()).replace('.', '_') + '.txt'
        f = open(fn, 'w')
        f.write('TYPE : TSP\n')
        f.write('DIMENSION : ' + str(len(handles)) + '\n')
        f.write('EDGE_WEIGHT_TYPE : EUC_3D\n')
        f.write('NODE_COORD_SECTION\n')
        node_to_key = dict()
        for i, key in enumerate(handles):
            f.write(str(i + 1) + ' ')
            for v in self.pointData[key]['pos']:
                f.write(str(int(round(v * factor))) + ' ')
            f.write('\n')
            node_to_key[i + 1] = key
        f.close()
        lis = lkh.solve(SOLVER_PATH, problem_file=fn, max_trials=10000, runs=10)
        os.remove(fn)
        return [node_to_key[i] for i in lis[0]]

    def make_goal_partition(self):
        """
        partitions the goals into a list for each blimp (unordered)
        @return: list of lists, length num_agents, total number of elements must be num_points without depot
                elements are keys of points for each agent to visit

        """
        # handles=self.pointData.keys()
        # H = [h for h in handles if not self.pointData[h]['is depot']]
        H = self.non_depot_handles
        kmeans = KMeans(n_clusters=self.num_agents, n_init=10).fit(np.array([self.pointData[h]['pos'] for h in H]))
        output = [[] for _ in range(self.num_agents)]

        for i, lab in enumerate(kmeans.labels_):
            output[lab].append(H[i])
        return (output)

    def get_disc_entropy_from_part(self, partition):
        goal_list = self.make_goal_list(partition)
        self.create_times(goal_list)
        cost = self.get_tot_disc_entropy(goal_list).item()
        return cost

class localSearchBlimp(travelingSalesBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 num_points,
                 spawn_pt_info,
                 alpha,
                 depot,
                 depot_tol=.1,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12, ):
        super().__init__(num_agents=num_agents, start_zone=start_zone, num_points=num_points,
                         spawn_pt_info=spawn_pt_info,
                         alpha=alpha, depot=depot, depot_tol=depot_tol, lr=lr, epochs=epochs,
                         grads_per_epoch=grads_per_epoch, converge_tol=converge_tol)

    def make_goal_partition(self):
        print('here')
        part = super().make_goal_partition()
        obj = self.get_disc_entropy_from_part(part)
        checked = {self.string_from_part(part)}
        while True:
            print('main loop')
            print(obj)

            print(self.string_from_part(part))
            print()
            best = None, None
            for neighbor in self.get_neighbors(part):
                string = self.string_from_part(neighbor)
                if string in checked: continue
                checked.add(string)
                ob = self.get_disc_entropy_from_part(neighbor)
                print(string)
                print(ob)
                if ob > obj and (best[1] is None or ob > best[1]):
                    best = (neighbor, ob)
            if best[0] is None:
                break
            part = best[0]
            obj = best[1]
        return part

    def string_from_part(self, part):
        s = []
        for h in self.non_depot_handles:
            for i, p in enumerate(part):
                if h in p:
                    s.append(i)
        return ''.join(str(ch) for ch in s)

    def part_from_string(self, string):
        part = [[] for _ in range(self.num_agents)]
        for i, h in enumerate(self.non_depot_handles):
            part[int(string[i])].append(h)
        return part

    def get_neighbors(self, partition):
        # returns all neighbors of partition attained by switching one element
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


class singleBlimp(travelingSalesBlimp):
    def __init__(self, start_zone, num_points, spawn_pt_info, alpha, depot, depot_tol=.1,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12, ):
        super().__init__(num_agents=1, start_zone=start_zone, num_points=num_points, spawn_pt_info=spawn_pt_info,
                         alpha=alpha, depot=depot, depot_tol=depot_tol, lr=lr, epochs=epochs,
                         grads_per_epoch=grads_per_epoch, converge_tol=converge_tol)


class doubleBlimp(travelingSalesBlimp):
    def __init__(self, start_zone, num_points, spawn_pt_info, alpha, depot, depot_tol=.1,
                 lr=.001,
                 epochs=10,
                 grads_per_epoch=float('inf'),
                 converge_tol=1E-12, ):
        super().__init__(num_agents=2, start_zone=start_zone, num_points=num_points, spawn_pt_info=spawn_pt_info,
                         alpha=alpha, depot=depot, depot_tol=depot_tol, lr=lr, epochs=epochs,
                         grads_per_epoch=grads_per_epoch, converge_tol=converge_tol)


points = 10
R = 2
num = 3
off = R * num / 2 + .5
depot = np.array((0, 0, 1))

"""
bb = travelingSalesBlimp(num,
                         lambda i: (i * 3 * R - off,
                                    0,
                                    R + 1),
                         num_points=num * points,
                         spawn_pt_and_time=lambda i: (
                             .25 * np.random.normal((0, 0, 0), (1, 10, 1)) +
                             np.array((R * np.cos(i * 2 * np.pi / points) + (3 * R * (i // points)) - off,
                                       0,
                                       1.5 + R + R * np.sin(i * 2 * np.pi / points))),
                             10),
                         scenePath=empty_path,
                         #wakeup=['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh']
                         )
"""
pounts = 10
infos = [{'pos': np.random.normal((1, 1, 0), (5, 5, 5)).clip([-10, -10, 1]),
          'tau': 1E0}
         for i in range(pounts)]
array = np.array
infos = [{'pos': array([-4.13007858, 2.21171989, 1.]), 'tau': 1.0},
         {'pos': array([-1.02735794, 3.71258206, 1.]), 'tau': 1.0},
         {'pos': array([-6.10099879, -2.66872272, 2.29806299]), 'tau': 1.0},
         {'pos': array([-1.44366215, 6.23788356, 1.]), 'tau': 1.0},
         {'pos': array([1.70957165, -2.35378516, 1.]), 'tau': 1.0},
         {'pos': array([-0.03658502, -2.71775426, 1.]), 'tau': 1.0},
         {'pos': array([-6.15198514, 10.884783, 1.]), 'tau': 1.0},
         {'pos': array([2.49765138, 7.08216976, 2.52365775]), 'tau': 1.0},
         {'pos': array([0.4479817, -4.1943272, 1.]), 'tau': 1.0},
         {'pos': array([-5.59681296, 0.53872593, 7.12257391]), 'tau': 1.0}]

bb = localSearchBlimp(num_agents=2,
                      start_zone=lambda i: depot + np.random.normal((0, 0, 0), (1, 1, 0)),
                      num_points=pounts,
                      spawn_pt_info=lambda i: infos[i],
                      alpha=1E-2,
                      grads_per_epoch=1000,
                      converge_tol=None,
                      depot=depot,
                      depot_tol=.5)
# print(bb.experiments(1, lambda t: bb.is_done()))
bb.init_exp(True)
handles = bb.pointData.keys()
H = [h for h in handles if not bb.pointData[h]['is depot']]
if False:
    pp = [[], []]
    for h in H:
        pp[int(np.random.random() * 1.1)].append(h)
    costs = []
    for part_index in range(int(2 ** len(H))):
        part_str = bin(2 ** len(H) + part_index)[3:]
        pp = [[], []]
        for i, h in enumerate(H):
            pp[int(part_str[i])].append(h)
        goal_list = bb.make_goal_list(pp)
        bb.create_times(goal_list)
        cost = bb.get_tot_disc_entropy(goal_list).item()
        costs.append(cost)
    print(costs)
    print(costs.index(max(costs)))
    print(costs[costs.index(max(costs))])
    quit()

H = [h for h in handles if not bb.pointData[h]['is depot']]
costs = []
part_index = 179
part_str = bin(2 ** len(H) + part_index)[3:]
pp = [[], []]
for i, h in enumerate(H):
    pp[int(part_str[i])].append(h)

for part in (bb.make_goal_partition(), pp):
    print('partition')
    print(part)
    print(bb.get_disc_entropy_from_part(part))
    goal_list = bb.make_goal_list(part)
    print('goal list')
    print(goal_list)
    bb.create_times(goal_list)
    cost = bb.get_tot_disc_entropy(goal_list).item()
    print('cost')
    print(cost)
    costs.append(cost)

    for key in bb.dwell_solution:
        print(key)
        for ele in bb.dwell_solution[key]:
            print(ele)
        print()
print(costs)
quit()
bb = doubleBlimp(lambda i: depot + np.random.normal((0, 0, 0), (1, 1, 0)),
                 num_points=10,
                 spawn_pt_info=lambda i: {
                     'pos': .25 * np.random.normal((0, 0, 0), (1, 10, 1)) +
                            np.array((R * np.cos(i * 2 * np.pi / points) + (3 * R * (i // points)) - off,
                                      0,
                                      1.5 + R + R * np.sin(i * 2 * np.pi / points))),
                     'tau': 1E0},
                 alpha=1E-3,
                 grads_per_epoch=2000,
                 converge_tol=None,
                 depot=np.array((-off, 0, R + 1)),
                 depot_tol=.5
                 )

print(bb.experiments(1, lambda t: bb.is_done()))
