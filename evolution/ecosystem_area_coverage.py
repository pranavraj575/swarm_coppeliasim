from src.network_blimps import *
from evolution.evolutionBlimp import EcosystemEvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running an area coverage ecosytem evolutionary experiment with height layers"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--obstacles", type=int, required=False, default=0,
                    help="number of obstacles to generate for training")
PARSER.add_argument("--obstacle_height", type=float, required=False, default=1.5,
                    help="obstacle spawn height, default of 1.5")

PARSER.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
PARSER.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

PARSER.add_argument("--height_layer", type=float, required=False, default=1.5,
                    help="layer to recognize a 'stacked' blimp")

PARSER.add_argument("--xmin", type=float, required=False, default=-3.,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
PARSER.add_argument("--xmax", type=float, required=False, default=3.,
                    help="x spawning upper bound")

PARSER.add_argument("--ymin", type=float, required=False, default=-3.,
                    help="y spawning upper bound")
PARSER.add_argument("--ymax", type=float, required=False, default=3.,
                    help="y spawning upper bound")

PARSER.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
PARSER.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

args = PARSER.parse_args()
check_basic(args=args)


class ecosystem_two_layer_area_coverage(k_tant_area_coverage):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfns,
                 height_range,
                 obstacles,
                 obstacle_height,
                 obstacle_paths,
                 end_time,
                 bounds,
                 height_cutoffs,
                 prop_ground=.8,
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
        @param networkfns: neural network function call for blimp to act
        @param height_range: R^2, height range to keep blimps at
        @param obstacles: number of obstacles to randomly spawn in
        @param obstacle_height: height to spawn in obstacles
        @param obstacle_paths: paths to obstacles to spawn in, list chosen from uniformly
        @param end_time: time it takes for experiment to end
        @param bounds: (RxR)^2, x bounds and y bounds to test for the area covered
            the goal function will uniformly choose some points in this area and rate the blimps based on closeness
        @param prop_ground: proportion of blimps that should be on 'ground level', used to determine number of cells
        @param height_factor: factor to multiply height adjust by
        @param height_cutoffs: segment the environment into cells based on this list of heights as dividers
            used for deployment engtropy fitness function
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
            networkfn=networkfns,
            height_range=height_range,
            use_ultra=True,
            obstacles=obstacles,
            obstacle_height=obstacle_height,
            obstacle_paths=obstacle_paths,
            end_time=end_time,
            bounds=bounds,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries
        )
        self.networks = self.network
        self.height_cutoffs = height_cutoffs
        self.dimension_split = int((num_agents*prop_ground)**(1/2))

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

    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: deployment entropy
            evolution/papers/Persistent Area Coverage for ...
        """
        entropy = 0
        boxes = np.array([
            [
                [0 for _ in range(self.dimension_split)]
                for _ in range(self.dimension_split)]
            for _ in range(len(self.height_cutoffs) + 1)])
        xbound, ybound = self.bounds
        for agent_id in self.agentData:
            xyz = self.get_position(agent_id, use_ultra=False)
            x, y, z = xyz
            xbox = self.dimension_split*(x - xbound[0])/(xbound[1] - xbound[0])
            ybox = self.dimension_split*(y - ybound[0])/(ybound[1] - ybound[0])
            zbox = 0
            for cut in self.height_cutoffs:
                if z > cut:
                    zbox += 1
            xbox = np.clip(xbox, 0, self.dimension_split - .5)
            ybox = np.clip(ybox, 0, self.dimension_split - .5)
            boxes[int(zbox), int(xbox), int(ybox)] += 1
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None

        for z in boxes:
            for zx in z:
                for zxy in zx:
                    p = zxy/self.num_agents
                    if p > 0:
                        entropy += p*np.log(1/p)
        return [entropy for _ in range(self.num_agents)]


OBS_DIR = os.path.join(DIR, 'models', 'obstacles')
OBS_PATHS = [os.path.join(OBS_DIR, d) for d in os.listdir(OBS_DIR)]

AGENTS = args.agents
END = args.end_time
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def ecosytem_exp_make(nets, sim=None, port=23000, wakeup=None):
    return ecosystem_two_layer_area_coverage(
        num_agents=AGENTS,
        start_zone=SPAWN_ZONE,
        scenePath=cage_arena_path,
        blimpPath=narrow_blimp_path,
        networkfns=lambda i: nets(i).activate,
        height_range=(h_low, h_upp),
        height_cutoffs=(args.height_layer,),
        obstacles=args.obstacles,
        obstacle_height=args.obstacle_height,
        obstacle_paths=OBS_PATHS,
        end_time=END,
        bounds=((-12.5, 12.5), (-12.5, 12.5)),
        height_factor=1.,
        sim=sim,
        simId=port,
        wakeup=wakeup,
        sleeptime=.01,
    )


def optimal_policy2d(inputs):
    k = len(inputs)
    vec = np.zeros(2)
    for i in range(k):
        angle = 2*np.pi*i/k + (np.pi/k)
        # angle that the neighbor is sensed at

        desired_angle = angle + np.pi
        # opposite direction
        temp = np.array((np.cos(desired_angle), np.sin(desired_angle)))
        vec += temp*inputs[i]
    vec = vec/np.linalg.norm(vec)
    return (vec + 1)/2  # since we need to output on [0,1]


save_name = 'ECOSYSTEM' + str(AGENTS) + '_blimp_' + \
            str(args.obstacles) + '_obstacle_area_coverage'
config_name = 'blimp_2d_area'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EcosystemEvolutionExperiment,
                   optimal_policy=PolicyWrapper(optimal_policy))
