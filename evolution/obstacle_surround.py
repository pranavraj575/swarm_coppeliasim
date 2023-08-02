from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running an obstacle surrounding evolutionary experiment"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--obstacles", type=int, required=False, default=1,
                    help="number of obstacles to generate for training")
PARSER.add_argument("--activation_range", type=float, required=False, default=3.,
                    help="distance where blimps are 'near' obstacles")

PARSER.add_argument("--cube", action="store_true", required=False,
                    help="whether to use the cube obstacle")

PARSER.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
PARSER.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

PARSER.add_argument("--xmin", type=float, required=False, default=-6.,
                    help="x spawning lower bound")
PARSER.add_argument("--xmax", type=float, required=False, default=6.,
                    help="x spawning upper bound")

PARSER.add_argument("--ymin", type=float, required=False, default=-6.,
                    help="y spawning upper bound")
PARSER.add_argument("--ymax", type=float, required=False, default=6.,
                    help="y spawning upper bound")

PARSER.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
PARSER.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

PARSER.add_argument("--xmin_obs", type=float, required=False, default=-8.,
                    help="x obstacle spawning lower bound")
PARSER.add_argument("--xmax_obs", type=float, required=False, default=8.,
                    help="x obstacle spawning upper bound")

PARSER.add_argument("--ymin_obs", type=float, required=False, default=-8.,
                    help="y obstacle spawning upper bound")
PARSER.add_argument("--ymax_obs", type=float, required=False, default=8.,
                    help="y obstacle spawning upper bound")

PARSER.add_argument("--obs_height", type=float, required=False, default=1.5,
                    help="obstacle spawn height")

args = PARSER.parse_args()
check_basic(args=args)
AGENTS = args.agents
END = args.end_time

h_low = args.height_lower
h_upp = args.height_upper
OBS_DIR = os.path.join(DIR, 'models', 'obstacles')
# OBS_PATHS = [os.path.join(OBS_DIR, d) for d in os.listdir(OBS_DIR)]
if args.cube:
    OBS_PATHS = [os.path.join(OBS_DIR, '3m_cube.ttm')]
else:
    OBS_PATHS = [os.path.join(OBS_DIR, '3d3_cylinder.ttm')]


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def expe_make(net, sim=None, port=23000, wakeup=None):
    return k_tant_obstacle_surround(
        num_agents=AGENTS,
        start_zone=SPAWN_ZONE,
        scenePath=cage_arena_path,
        blimpPath=narrow_blimp_path,
        networkfn=net.activate,
        height_range=(h_low, h_upp),
        obstacles=args.obstacles,
        obstacle_paths=OBS_PATHS,
        obstacle_zone=((args.xmin_obs, args.xmax_obs), (args.ymin_obs, args.ymax_obs), args.obs_height),
        activation_range=args.activation_range,
        end_time=END,
        height_factor=1.,
        sim=sim,
        simId=port,
        wakeup=wakeup,
        sleeptime=.01,
    )


def optimal_policy(inputs):
    return np.zeros(2)


save_name = str(AGENTS) + '_blimp_' \
            + str(args.obstacles) + ('_cube' if args.cube else '_cylinder') + '_obstacle_surround' + \
            '_range_' + str(args.activation_range).replace('.', '_')
config_name = 'blimp_' + str(args.obstacles) + '_obs_surround'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment,
                   optimal_policy=optimal_policy)
