from src.maze_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment
from evolution.arg_parser import *
from evolution.ev_utils import *

PARSER.description = "for creating and running maze evolutionary experiments with distance sensing instead of number of neighbors"

PARSER.add_argument("--end_time", type=float, required=False, default=60.,
                    help="time to end the experiment")

PARSER.add_argument("--height", type=int, required=False, default=5,
                    help="height of maze")
PARSER.add_argument("--width", type=int, required=False, default=5,
                    help="width of maze")

PARSER.add_argument("--trials_fun", action='store', required=False, default='min',
                    help="take this action over the trials (from --goal_fun <min, max, mean>)")

args = PARSER.parse_args()
check_basic(args=args)

trf = args.trials_fun
if trf == 'max':
    trials_fun = np.max
elif trf == 'min':
    trials_fun = np.min
elif trf == 'mean':
    trials_fun = np.min
else:
    raise Exception('--trials_fun must be from ["max", "min", "mean"]')

AGENTS = args.agents
END = args.end_time
H = args.height
W = args.width
if args.trials == 1:
    args.trials = 2
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

    return wall_dist_sense_max_amazing_blimp(num_agents=AGENTS,
                                        scenePath=maze_view_path,
                                        blimpPath=narrow_blimp_path,
                                        networkfn=net.activate,
                                        end_time=END,
                                        start_squares=3,
                                        trials_fun=trials_fun,
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
                                        sleeptime=.01,
                                        )


def optimal_policy(inputs):
    neighbor_behavior=.6
    k = len(inputs)-4
    vec_neigh = np.zeros(2)
    for i in range(k):
        angle = 2*np.pi*i/k + (np.pi/k)
        # angle that the neighbor is sensed at

        desired_angle = angle + np.pi
        # opposite direction

        temp = np.array((np.cos(desired_angle), np.sin(desired_angle)))
        vec_neigh += temp*inputs[i]
    vec_neigh = vec_neigh/np.linalg.norm(vec_neigh)
    vec_walls=np.zeros(2)
    for i in range(4):
        wall=inputs[len(inputs)-4+i]
        dir=np.array((np.cos(i*np.pi/2),np.sin(i*np.pi/2)))
        if wall>0:
            vec_walls+=dir
    vec_walls=vec_walls/(np.linalg.norm(vec_walls) if np.linalg.norm(vec_walls)>0 else 1)
    vec=vec_walls(1-neighbor_behavior)+vec_neigh*neighbor_behavior
    return (vec + 1)/2  # since we need to output on [0,1]


save_name = str(AGENTS) + '_blimp_' + str(H) + 'x' + str(W) + 'maze_' + trf + '_of_trials_dist_wall_sensing'
config_name = 'blimp_maze_wall_sense'

experiment_handler(args=args,
                   save_name=save_name,
                   config_name=config_name,
                   exp_maker=expe_make,
                   Constructor=EvolutionExperiment,
                   optimal_policy=PolicyWrapper(optimal_policy))