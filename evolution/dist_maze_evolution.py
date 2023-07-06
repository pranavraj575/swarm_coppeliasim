from src.maze_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
import argparse

parser = argparse.ArgumentParser(description="for creating and running maze evolutionary experiments")
parser.add_argument("-a", "--agents", type=int, required=False, default=20,
                    help="Specify number of agents")
parser.add_argument("-g", "--generations", type=int, required=False, default=0,
                    help="generations to train for")
parser.add_argument("--create", action="store_true", required=False,
                    help="whether to create new directory")
parser.add_argument("--height", type=int, required=False, default=5,
                    help="height of maze")
parser.add_argument("--width", type=int, required=False, default=5,
                    help="width of maze")
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
H = args.height
W = args.width
DIFF = True
ALWAYS_DOWN = False


def expe_make(net, sim=None, port=23000, wakeup=None):
    def make_maze():
        ENTRY = (0, np.random.random())
        EXIT = (1, np.random.random())
        if not ALWAYS_DOWN:
            if np.random.random() < .5:  # opposite direction
                ENTRY = 1, ENTRY[1]
                EXIT = 0, EXIT[1]
            if np.random.random() < .5:  # swap xy
                ENTRY = ENTRY[1], ENTRY[0]
                EXIT = EXIT[1], EXIT[0]
        ENTRY = min(int(ENTRY[0]*H), H - 1), min(int(ENTRY[1]*W), W - 1)
        EXIT = min(int(EXIT[0]*H), H - 1), min(int(EXIT[1]*W), W - 1)

        if DIFF:
            while EXIT[0] == ENTRY[0]:
                EXIT = (np.random.randint(0, H), EXIT[1])
            while EXIT[1] == ENTRY[1]:
                EXIT = (EXIT[0], np.random.randint(0, W))
        mm = Maze(H, W, entry=ENTRY, exit=EXIT)
        entry = mm.entry_coor
        ext = mm.exit_coor
        orientations = []
        walls = []
        for cell in entry, ext:
            wall = None
            orientation = None
            if cell[0] == 0 and not mm.initial_grid[cell[0]][cell[1]].walls['top']:
                wall = 'top'
                orientation = np.pi
            if cell[1] == 0 and not mm.initial_grid[cell[0]][cell[1]].walls['left']:
                wall = 'left'
                orientation = 3*np.pi/2
            if cell[0] == H - 1 and not mm.initial_grid[cell[0]][cell[1]].walls['bottom']:
                wall = 'bottom'
                orientation = 0
            if cell[1] == W - 1 and not mm.initial_grid[cell[0]][cell[1]].walls['right']:
                wall = 'right'
                orientation = np.pi/2
            orientations.append(orientation)
            walls.append(wall)
        return {'maze': mm,
                'entry': entry,
                'exit': ext,
                'entry_wall': walls[0],
                'entry_orientation': (0, 0, orientations[0]),
                # 'exit_wall': walls[1],
                # 'exit_orientation': (0, 0, orientations[1]),
                }

    return dist_sense_max_amazing_blimp(num_agents=AGENTS,
                           scenePath=maze_view_path,
                           blimpPath=narrow_blimp_path,
                           networkfn=net.activate,
                           end_time=END,
                           grid_size=2,
                           maze_entry_gen=make_maze,
                           wall_spawn_height=1.5,
                           wall_dir=wall_path,
                           height_range=(1, 1),
                           height_factor=1.,
                           use_ultra=True,
                           sim=sim,
                           simId=port,
                           wakeup=wakeup,
                           sleeptime=.01
                           )


save_name = str(AGENTS) + '_blimp_' + str(H) + 'x' + str(W) + 'maze_max_goal_dist_sensing'
checkpt_dir = os.path.join(DIR, 'checkpoints', save_name)

print("SAVING TO:", checkpt_dir)
if not os.path.exists(checkpt_dir):
    if args.create:
        os.makedirs(checkpt_dir)
    else:
        raise Exception("DIRECTORY DOES NOT EXIST (try running with --create): " + checkpt_dir)
ee = EvolutionExperiment(checkpt_dir=checkpt_dir,
                         exp_maker=expe_make,
                         config_name='blimp_maze')
if gens:
    port_step = args.port_step
    zmq_def_port = 23000 + port_step*args.offset
    websocket_def_port = 23050 + port_step*args.offset

    ee.train(generations=gens,
             TRIALS=2,
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
    print(ee.result_of_experiment())