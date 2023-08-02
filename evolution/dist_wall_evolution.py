from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.descripton = "for setting up a wall climbing evolutionary experiment with distance sensing instead of local number of neighbors"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
PARSER.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

PARSER.add_argument("--xmin", type=float, required=False, default=.5,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
PARSER.add_argument("--xmax", type=float, required=False, default=11.,
                    help="x spawning upper bound")

PARSER.add_argument("--ymin", type=float, required=False, default=-7.,
                    help="y spawning upper bound")
PARSER.add_argument("--ymax", type=float, required=False, default=7.,
                    help="y spawning upper bound")

PARSER.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
PARSER.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

args = PARSER.parse_args()
check_basic(args=args)
AGENTS = args.agents
gens = args.generations
END = args.end_time
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def expe_make(net, sim=None, port=23000, wakeup=None):
    return dist_k_tant_wall_climb_blimp(num_agents=AGENTS,
                                        start_zone=SPAWN_ZONE,
                                        scenePath=caged_wall_climb_path,
                                        blimpPath=narrow_blimp_path,
                                        networkfn=net.activate,
                                        end_time=END,
                                        height_range=(h_low, h_upp),
                                        use_ultra=True,
                                        height_factor=1.,
                                        sim=sim,
                                        simId=port,
                                        wakeup=wakeup,
                                        sleeptime=.01
                                        )


def optimal_policy(inputs):
    return np.zeros(2)


save_name = str(AGENTS) + '_blimp_height_' + str(h_low).replace('.', '_') + "_to_" + str(h_upp).replace('.', '_') + \
            '_wall_climb_neighbor_dist_sensing'
config_name = 'blimp_wall'
experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment,
                   optimal_policy=optimal_policy)
