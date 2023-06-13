from src.swarm_expiriment import *
from collections import defaultdict
import lkh
import time
SOLVER_PATH = '/home/rajbhandari/LKH-3.0.6/LKH'


class travelingSalesBlimp(BlimpExperiment):
    def __init__(self, num_agents,
                 start_zone,
                 num_points,
                 spawn_pt_and_time,
                 sim=None,
                 wakeup=None,
                 sleeptime=1,
                 scenePath=empty_path,
                 blimpPath=narrow_blimp_path,
                 goalPath="/home/rajbhandari/projects/blimp_coppeliasim/models/goal.ttm",
                 speed=.5,
                 tolerance=.2
                 ):
        super().__init__(num_agents=num_agents,
                         start_zone=start_zone,
                         sim=sim,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         scenePath=scenePath,
                         blimpPath=blimpPath)
        self.num_points = num_points
        self.spawn_pt_and_time = spawn_pt_and_time
        self.goalPath = goalPath
        self.pointData = defaultdict(lambda: dict())
        self.speed = speed
        self.tolerance = tolerance
        self.visited = []
        self.waiting = dict()
        self.goal_list = None

    ####################################################################################################################
    # init/shutdown functions (needs implementation in subclass)
    ####################################################################################################################
    def spawnThings(self):
        super().spawnThings()
        self.pointData = defaultdict(lambda: dict())
        for i in range(self.num_points):
            pt, hang_time = self.spawn_pt_and_time(i)
            goalHandle = self.spawnBlimp(self.goalPath, pt, 1)
            # self.points.append((pt, goalHandle))
            self.pointData[goalHandle]['pos'] = pt
            self.pointData[goalHandle]['time'] = hang_time
            self.pointData[goalHandle]['order_added'] = i
            self.pointData[goalHandle]['edge_cost'] = dict()
            self.set_color(self.find_handle(pt), [255, 0, 0])
        self.visited = []
        self.waiting = dict()
        self.goal_list = self.make_goal_list()

    def _cost_fn(self, hand1, hand2):
        return np.linalg.norm(self.pointData[hand1]['pos'] - self.pointData[hand2]['pos'])

    def get_cost(self, pt1, pt2):
        hand1 = self.find_handle(pt1)
        hand2 = self.find_handle(pt2)
        if hand2 not in self.pointData[hand1]['edge_cost']:
            cost = self._cost_fn(hand1, hand2)
            self.pointData[hand1]['edge_cost'][hand2] = cost
        return self.pointData[hand1]['edge_cost'][hand2]

    def make_goal_list(self):
        """
        https://pypi.org/project/lkh/
        http://webhotel4.ruc.dk/~keld/research/LKH-3/LKH-3_REPORT.pdf
        gives each blimp a list of points to go to
        @return: list of lists, length num_agents, total number of elements must be num_points
                elements are keys of points for each agent to visit
        """
        print("WARNING: badly implemented")
        output = []
        for i in range(self.num_agents):
            low = int(np.ceil(i * self.num_points / self.num_agents))
            high = int(np.ceil((i + 1) * self.num_points / self.num_agents))
            temp = []
            for k in range(low, high):
                for key in self.pointData:
                    if self.pointData[key]['order_added'] == k:
                        temp.append(key)
            # output.append([p for p, _ in self.points[low:high]])
            output.append(temp)
        return output

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
            goal = None
            for g_hand in goals:
                g = self.pointData[g_hand]['pos']
                if g_hand not in self.visited:
                    goal = g
                    if np.linalg.norm(g - pos) <= self.tolerance:
                        # reached
                        if g_hand in self.waiting:
                            spent = self.sim.getSimulationTime() - self.waiting[g_hand]
                            if spent >= self.pointData[g_hand]['time']:
                                self.waiting.pop(g_hand)
                                self.visited.append(g_hand)
                                self.set_color(self.find_handle(g), [0, 255, 0])
                        else:
                            self.waiting[g_hand] = self.sim.getSimulationTime()
                    else:
                        if g_hand in self.waiting:
                            self.waiting.pop(g_hand)
                    break

            if goal is None:
                vec = np.zeros(3)
            else:
                vec = (goal - pos) * .5
                if np.linalg.norm(vec) > self.speed:
                    vec = self.speed * vec / np.linalg.norm(vec)
            self.move_agent(agent_id, vec)

    def goal_data(self):
        return None


class lkhSaleBlimp(travelingSalesBlimp):
    def __init__(self, start_zone, num_points, spawn_pt_and_time):
        super().__init__(1, start_zone, num_points, spawn_pt_and_time)

    def make_goal_list(self, factor=1000):
        """
        creates the order of the hamiltonian path according to lkh
        @param factor: lkh takes integers, so we multiply coordinates by factor to increase precision
        @return:[list of handles for agent to visit]
        """
        fn = 'temp'+str(time.time()).replace('.','_')+'.txt'
        f = open(fn, 'w')
        f.write('TYPE : TSP\n')
        f.write('DIMENSION : ' + str(self.num_points) + '\n')
        f.write('EDGE_WEIGHT_TYPE : EUC_3D\n')
        f.write('NODE_COORD_SECTION\n')
        node_to_key = dict()
        for i, key in enumerate(self.pointData):
            f.write(str(i + 1) + ' ')
            for v in self.pointData[key]['pos']:
                f.write(str(int(round(v * factor))) + ' ')
            f.write('\n')
            node_to_key[i + 1] = key
        f.close()
        lis = lkh.solve(SOLVER_PATH, problem_file=fn, max_trials=10000, runs=10)
        os.remove(fn)
        return [[node_to_key[i] for i in lis[0]]]


points = 10
R = 2
num = 3
off = R * num / 2 + .5

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
bb = lkhSaleBlimp(lambda i: (i * 3 * R - off,
                             0,
                             R + 1),
                  num_points=10,
                  spawn_pt_and_time=lambda i: (
                            .25 * np.random.normal((0, 0, 0), (1, 10, 1)) +
                            np.array((R * np.cos(i * 2 * np.pi / points) + (3 * R * (i // points)) - off,
                                      0,
                                      1.5 + R + R * np.sin(i * 2 * np.pi / points))),
                            0),
                  )
print(bb.experiments(1, lambda t: len(bb.visited) == bb.num_points))
