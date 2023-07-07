from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *

PARSER.descripton = "for setting up a wall climbing evolutionary experiment with distance sensing instead of local number of neighbors"

PARSER.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
PARSER.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

PARSER.add_argument("--xmin", type=float, required=False, default=.5,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
PARSER.add_argument("--xmax", type=float, required=False, default=11.,
                    help="x spawning upper bound")

PARSER.add_argument("--ymin", type=float, required=False, default=-9.,
                    help="y spawning upper bound")
PARSER.add_argument("--ymax", type=float, required=False, default=9.,
                    help="y spawning upper bound")

PARSER.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
PARSER.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

args = PARSER.parse_args()
check_basic(args=args)
AGENTS = args.agents
gens = args.generations
END = 60
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


save_name = str(AGENTS) + '_blimp_height_' + str(h_low).replace('.', '_') + "_to_" + str(h_upp).replace('.', '_') + \
            '_wall_climb_neighbor_dist_sensing'
checkpt_dir = os.path.join(DIR, 'checkpoints', save_name)
print("SAVING TO:", checkpt_dir)

if not os.path.exists(checkpt_dir):
    if args.create:
        os.makedirs(checkpt_dir)
    else:
        raise Exception("DIRECTORY DOES NOT EXIST (try running with --create): " + checkpt_dir)
ee = EvolutionExperiment(checkpt_dir=checkpt_dir,
                         exp_maker=expe_make,
                         config_name='blimp_wall')
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
             num_sim_range=None if args.sims_low < 1 else (args.sims_low, args.sims_high)
             )

if args.show_stats:
    ee.show_stats()
if args.show:
    print(ee.result_of_experiment())
