from src.network_ankis import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *

PARSER.description = "for creating and running anki test experiment"

PARSER.add_argument("--xmin", type=float, required=False, default=-.25,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
PARSER.add_argument("--xmax", type=float, required=False, default=.25,
                    help="x spawning upper bound")

PARSER.add_argument("--ymin", type=float, required=False, default=-.25,
                    help="y spawning upper bound")
PARSER.add_argument("--ymax", type=float, required=False, default=.25,
                    help="y spawning upper bound")

PARSER.add_argument("--zspawn", type=float, required=False, default=.035,
                    help="z spawning")

args = PARSER.parse_args()
check_basic(args=args)
AGENTS = args.agents
gens = args.generations
END = 60


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), args.zspawn


def expe_make(net, sim=None, port=23000, wakeup=None):
    return k_tant_anki_area_coverage(num_agents=AGENTS,
                                     start_zone=SPAWN_ZONE,
                                     scenePath=anki_arena_path,
                                     ankiPath=anki_path,
                                     networkfn=net.activate,
                                     bounds=((-.45, .6), (-.5, .5)),
                                     end_time=END,
                                     sim=sim,
                                     simId=port,
                                     wakeup=wakeup,
                                     sleeptime=.01
                                     )


save_name = str(AGENTS) + '_anki_area_coverage'

checkpt_dir = os.path.join(DIR, 'checkpoints', save_name)
print("SAVING TO:", checkpt_dir)

if not os.path.exists(checkpt_dir):
    if args.create:
        os.makedirs(checkpt_dir)
    else:
        raise Exception("DIRECTORY DOES NOT EXIST (try running with --create): " + checkpt_dir)
ee = EvolutionExperiment(checkpt_dir=checkpt_dir,
                         exp_maker=expe_make,
                         config_name='anki_area')
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
    print(ee.result_of_experiment(gen_indices=(args.show_gen,)))
