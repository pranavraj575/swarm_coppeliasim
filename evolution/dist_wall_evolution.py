from src.network_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
import argparse

parser = argparse.ArgumentParser(description="for creating and running wall climbing evolutionary experiments")
parser.add_argument("-a", "--agents", type=int, required=False, default=20,
                    help="Specify number of agents")
parser.add_argument("-g", "--generations", type=int, required=False, default=0,
                    help="generations to train for")
parser.add_argument("--create", action="store_true", required=False,
                    help="whether to create new directory")

parser.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
parser.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

parser.add_argument("--xmin", type=float, required=False, default=.5,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
parser.add_argument("--xmax", type=float, required=False, default=11.,
                    help="x spawning upper bound")

parser.add_argument("--ymin", type=float, required=False, default=-9.,
                    help="y spawning upper bound")
parser.add_argument("--ymax", type=float, required=False, default=9.,
                    help="y spawning upper bound")

parser.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
parser.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

parser.add_argument("--num_sims", type=int, required=False, default=8,
                    help="number of simulators to use for training")
parser.add_argument("--sims_low", type=int, required=False, default=-1,
                    help="low bound of num_sims to try")
parser.add_argument("--sims_high", type=int, required=False, default=-1,
                    help="high bound of num_sims to try")
parser.add_argument("--offset", type=int, required=False, default=0,
                    help="offset port number (should be number of simulators already in use)")
parser.add_argument("--port_step", type=int, required=False, default=2,
                    help="ports to skip for each new coppeliasim instance")
parser.add_argument("--overwrite", action="store_true", required=False,
                    help="whether to overwrite start instead of starting at recent checkpoint")
parser.add_argument("--show", action="store_true", required=False,
                    help="whether to show stats of all gens")
parser.add_argument("--show_result", action="store_true", required=False,
                    help="whether to show result at end")
args = parser.parse_args()
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
    return dist_sense_xy_wall_climb_blimp(num_agents=AGENTS,
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
if args.show:
    ee.show_stats()
if args.show_result:
    print(ee.result_of_experiment(gen_indices=[0, 1, 2, 3]))
