from src.network_ankis import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running anki test experiment"

PARSER.add_argument("--end_time", type=float, required=False, default=20.,
                    help="time to end the experiment")

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
END = args.end_time


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), args.zspawn


def expe_make(net, sim=None, port=23000, wakeup=None):
    return LRAngleAnki(num_agents=AGENTS,
                       start_zone=SPAWN_ZONE,
                       scenePath=anki_arena_path,
                       ankiPath=anki_path,
                       networkfn=net.activate,
                       angle_goal=np.pi,
                       end_time=END,
                       sim=sim,
                       simId=port,
                       wakeup=wakeup,
                       sleeptime=.01
                       )


save_name = str(AGENTS) + '_anki_test'
config_name = 'anki_test'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment)
