from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running an area coverage evolutionary experiment, can sense wall"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--obstacles", type=int, required=False, default=0,
                    help="number of obstacles to generate for training")
PARSER.add_argument("--obstacle_height", type=float, required=False, default=1.5,
                    help="obstacle spawn height, default of 1.5 (only relevant for 2d)")

PARSER.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at (irrelevant for 3D)")
PARSER.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at (irrelevant for 3D)")

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
OBS_DIR = os.path.join(DIR, 'models', 'obstacles')
OBS_PATHS = [os.path.join(OBS_DIR, d) for d in os.listdir(OBS_DIR)]

AGENTS = args.agents
END = args.end_time
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def expe_make(net, sim=None, port=23000, wakeup=None):
    return k_tant_wall_sense_area_coverage(
        num_agents=AGENTS,
        start_zone=SPAWN_ZONE,
        scenePath=cage_arena_path,
        blimpPath=narrow_blimp_path,
        networkfn=net.activate,
        height_range=(h_low, h_upp),
        use_ultra=False,
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


def optimal_policy(inputs):
    return np.zeros(2)


save_name = str(AGENTS) + '_blimp_' \
            + str(args.obstacles) + '_obstacle_area_coverage_wall_sense'
config_name = 'blimp_2d_area'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment,
                   optimal_policy=optimal_policy)
