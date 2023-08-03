from src.network_ankis import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running anki test experiment"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--large", action="store_true", required=False,
                    help="whether to use 'largeAnkiArena'")

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
LARGE = args.large
if LARGE:
    bounds = ((-.9, 1.2), (-1, 1))
    SCENE = anki_large_arena_path
else:
    bounds = ((-.45, .6), (-.5, .5))
    SCENE = anki_arena_path


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), args.zspawn


def expe_make(net, sim=None, port=23000, wakeup=None):
    return k_tant_anki_area_coverage(num_agents=AGENTS,
                                     start_zone=SPAWN_ZONE,
                                     scenePath=SCENE,
                                     ankiPath=anki_path,
                                     networkfn=net.activate,
                                     bounds=bounds,
                                     end_time=END,
                                     sim=sim,
                                     simId=port,
                                     wakeup=wakeup,
                                     sleeptime=.01
                                     )


def optimal_policy(inputs):
    k = len(inputs) - 1
    vec = np.zeros(2)
    for i in range(k):
        angle = 2*np.pi*i/k + (np.pi/k)
        # angle that the neighbor is sensed at

        desired_angle = angle + np.pi
        # opposite direction

        temp = np.array((np.cos(desired_angle), np.sin(desired_angle)))
        vec += temp*min(1, inputs[i + 1])/k
    vec = vec/np.linalg.norm(vec)
    forward = vec[0]
    left = vec[1]
    L = forward - left
    R = forward + left
    return (np.array((L, R)) + 1)/2


save_name = str(AGENTS) + '_anki_area_coverage' + ('_large' if LARGE else '')
config_name = 'anki_area'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment,
                   optimal_policy=PolicyWrapper(optimal_policy))
