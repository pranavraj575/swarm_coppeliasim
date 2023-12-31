from src.network_blimps import *
from evolution.evolutionBlimp import EcosystemEvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *


class ecosystem_xy_wall_climb(k_tant_wall_climb_blimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfns,
                 height_range,
                 use_ultra,
                 end_time,
                 rng=2,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        ecosystem implementation of blimp wall climb
            this way is much easier than redefining everything as a ecosystemBlimpNet class
            the methods that need changing are goal_data and step

        """
        super().__init__(num_agents=num_agents,
                         start_zone=start_zone,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         networkfn=networkfns,
                         height_range=height_range,
                         use_ultra=use_ultra,
                         end_time=end_time,
                         rng=rng,
                         height_factor=height_factor,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries
                         )
        self.networks = self.network

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
        val = super().goal_data()
        if val is None:
            return None
        return [val for _ in range(self.num_agents)]


PARSER.description = "for testing the ecosystem setup on a basic wall climbing evolutionary experiment"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
PARSER.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

PARSER.add_argument("--range", type=float, required=False, default=5.,
                    help="range to detect neighbors")

PARSER.add_argument("--xmin", type=float, required=False, default=.5,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
PARSER.add_argument("--xmax", type=float, required=False, default=8.,
                    help="x spawning upper bound")

PARSER.add_argument("--ymin", type=float, required=False, default=-4.,
                    help="y spawning upper bound")
PARSER.add_argument("--ymax", type=float, required=False, default=4.,
                    help="y spawning upper bound")

PARSER.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
PARSER.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

args = PARSER.parse_args()
check_basic(args=args)
AGENTS = args.agents
RANGE = args.range
END = args.end_time
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def eco_expe_make(nets, sim=None, port=23000, wakeup=None):
    return ecosystem_xy_wall_climb(num_agents=AGENTS,
                                   start_zone=SPAWN_ZONE,
                                   scenePath=caged_wall_climb_path,
                                   blimpPath=narrow_blimp_path,
                                   networkfns=lambda i: nets(i).activate,
                                   end_time=END,
                                   rng=RANGE,
                                   height_range=(h_low, h_upp),
                                   use_ultra=True,
                                   height_factor=1.,
                                   sim=sim,
                                   simId=port,
                                   wakeup=wakeup,
                                   sleeptime=.01)


def optimal_policy(inputs):
    return np.zeros(2)


save_name = 'ECOSYSTEM' + str(AGENTS) + \
            '_blimp_height_' + str(h_low).replace('.', '_') + "_to_" + str(h_upp).replace('.', '_') + \
            '_wall_climb_neighbor_rng_' + str(RANGE).replace('.', '_')
config_name = 'blimp_wall_test'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EcosystemEvolutionExperiment,
                   optimal_policy=PolicyWrapper(optimal_policy))
