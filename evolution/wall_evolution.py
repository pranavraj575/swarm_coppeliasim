from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running wall climbing evolutionary experiments"

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
RANGE = args.range
END = args.end_time
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def expe_make(net, sim=None, port=23000, wakeup=None):
    return k_tant_wall_climb_blimp(num_agents=AGENTS,
                                   start_zone=SPAWN_ZONE,
                                   scenePath=caged_wall_climb_path,
                                   blimpPath=narrow_blimp_path,
                                   networkfn=net.activate,
                                   end_time=END,
                                   rng=RANGE,
                                   height_range=(h_low, h_upp),
                                   use_ultra=True,
                                   height_factor=1.,
                                   sim=sim,
                                   simId=port,
                                   wakeup=wakeup,
                                   sleeptime=.01
                                   )


def optimal_policy(inputs):
    goal_seeking = .6
    k = len(inputs)
    vec = np.zeros(2)
    for i in range(k):
        angle = 2*np.pi*i/k + (np.pi/k)
        # angle that the neighbor is sensed at

        desired_angle = angle
        # same direction

        temp = np.array((np.cos(desired_angle), np.sin(desired_angle)))
        vec += temp*inputs[i]

    vec = safe_linalg_normalize(vec)*(1 - goal_seeking)
    vec += goal_seeking*np.array((-1, 0))
    return (vec + 1)/2  # since we need to output on [0,1]


save_name = str(AGENTS) + '_blimp_height_' + str(h_low).replace('.', '_') + "_to_" + str(h_upp).replace('.', '_') + \
            '_wall_climb_neighbor_rng_' + str(RANGE).replace('.', '_')
config_name = 'blimp_wall'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment,
                   optimal_policy=PolicyWrapper(optimal_policy))
