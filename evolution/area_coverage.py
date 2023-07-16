from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *

PARSER.description = "for creating and running an area coverage evolutionary experiment"

PARSER.add_argument("--three_d", action="store_true", required=False,
                    help="whether to use the 3d case")

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
gens = args.generations
if args.sims_low >= 1:
    if not args.sims_low <= args.num_sims or not args.num_sims < args.sims_high:
        raise Exception("bruh")
END = 60
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def expe_make(net, sim=None, port=23000, wakeup=None):
    if args.three_d:
        return l_k_tant_area_coverage(
            num_agents=AGENTS,
            start_zone=SPAWN_ZONE,
            scenePath=cage_arena_path,
            blimpPath=narrow_blimp_path,
            networkfn=net.activate,
            bounds=((-12.5, 12.5), (-12.5, 12.5), (0., 25.)),
            obstacles=args.obstacles,
            obstacle_paths=OBS_PATHS,
            end_time=END,
            use_ultra=True,
            sim=sim,
            simId=port,
            wakeup=wakeup,
            sleeptime=.01
        )
    else:
        return k_tant_area_coverage(
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


dim = ('3' if args.three_d else '2')

save_name = str(AGENTS) + '_blimp_' + dim + 'D_' \
            + str(args.obstacles) + '_obstacle_area_coverage'
checkpt_dir = os.path.join(DIR, 'checkpoints', save_name)
print("SAVING TO:", checkpt_dir)

if not os.path.exists(checkpt_dir):
    if args.create:
        os.makedirs(checkpt_dir)
    else:
        raise Exception("DIRECTORY DOES NOT EXIST (try running with --create): " + checkpt_dir)
ee = EvolutionExperiment(checkpt_dir=checkpt_dir,
                         exp_maker=expe_make,
                         config_name='blimp_' + dim + 'd_area')
if gens:
    port_step = args.port_step
    zmq_def_port = 23000 + port_step*args.offset
    websocket_def_port = 23050 + port_step*args.offset

    ee.train(generations=gens,
             TRIALS=1,
             num_simulators=args.num_sims,
             headless=not args.show,
             restore=not args.overwrite,
             evaluate_each_gen=True,
             zmq_def_port=zmq_def_port,
             websocket_def_port=websocket_def_port,
             port_step=port_step,
             num_sim_range=None if args.sims_low < 1 else (args.sims_low, args.sims_high),
             debug=args.debug
             )

if args.show_stats:
    ee.show_stats()
if args.show:
    print(ee.result_of_experiment(gen_indices=(args.show_gen,)))
