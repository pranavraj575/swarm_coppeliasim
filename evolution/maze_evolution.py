from src.maze_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
import argparse

parser = argparse.ArgumentParser(description="for creating and running maze evolutionary experiments")
parser.add_argument("-a", "--agents", type=int, required=False, default=20,
                    help="Specify number of agents")
parser.add_argument("-g", "--generations", type=int, required=False, default=0,
                    help="generations to train for")
parser.add_argument("--num_sims", type=int, required=False, default=8,
                    help="number of simulators to use for training")
parser.add_argument("--offset", type=int, required=False, default=0,
                    help="offset port number (should be number of simulators already in use)")
parser.add_argument("--port_step", type=int, required=False, default=2,
                    help="ports to skip for each new coppeliasim instance")
parser.add_argument("--overwrite", action="store_true", required=False,
                    help="whether to overwrite start instead of starting at recent checkpoint")
parser.add_argument("--show", action="store_true", required=False,
                    help="whether to show result at end")
args = parser.parse_args()
AGENTS = args.agents
gens = args.generations

END = 60
H = 5
W = 5


def expe_make(net, sim=None, port=23000, wakeup=None):
    ENTRY = (0, np.random.randint(0, W))

    CENTER = np.array((1 + 2*ENTRY[1], 5))
    R = 2.7

    def START_ZONE(i):
        out = CENTER - 2*R
        while np.linalg.norm(out - CENTER) > R:
            out = CENTER + np.random.uniform(-R, R, 2)
        return (out[0], out[1], 1)

    def make_maze():
        EXIT = (H - 1, np.random.randint(0, W))
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

    return maxAmazingBlimp(num_agents=AGENTS,
                        start_zone=START_ZONE,
                        scenePath=maze_view_path,
                        blimpPath=narrow_blimp_path,
                        networkfn=net.activate,
                        end_time=END,
                        grid_size=2,
                        maze_entry_gen=make_maze,
                        wall_spawn_height=1.5,
                        wall_dir=wall_path,
                        height_range=(1, 1),
                        use_ultra=True,
                        sim=sim,
                        simId=port,
                        wakeup=wakeup,
                        sleeptime=.01
                        )


ee = EvolutionExperiment(name=str(AGENTS) + '_blimp_' + str(H) + 'x' + str(W) + 'maze_max_goal',
                         exp_maker=expe_make,
                         config_name='blimp_maze')
if gens:
    port_step = args.port_step
    zmq_def_port = 23000 + port_step*args.offset
    websocket_def_port = 23050 + port_step*args.offset

    ee.train(generations=gens,
             TRIALS=2,
             num_simulators=args.num_sims,
             headless=True,
             restore=not args.overwrite,
             evaluate_each_gen=True,
             zmq_def_port=zmq_def_port,
             websocket_def_port=websocket_def_port
             )
if args.show:
    ee.result_of_experiment(search_all_gens=False)
