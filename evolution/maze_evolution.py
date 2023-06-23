from src.maze_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment

AGENTS=20
END=60
H=5
W=5
def expe_make(net, sim=None, port=23000, wakeup=None):
    ENTRY = (0, np.random.randint(0, W))
    EXIT = (H-1, np.random.randint(0, W))

    CENTER = np.array((1 + 2*ENTRY[1], 5))
    R = 2.7

    def START_ZONE(i):
        out = CENTER - 2*R
        while np.linalg.norm(out - CENTER) > R:
            out = CENTER + np.random.uniform(-R, R, 2)
        return (out[0], out[1], 1)

    def make_maze():
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
                #'exit_wall': walls[1],
                #'exit_orientation': (0, 0, orientations[1]),
                }

    return amazingBlimp(num_agents=AGENTS,
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


ee = EvolutionExperiment(name=str(AGENTS)+'_blimp_'+str(H)+'x'+str(W)+'maze',
                         exp_maker=expe_make,
                         config_name='blimp_maze')
ee.train(generations=5,
         TRIALS=1,
         num_simulators=8,
         headless=True,
         restore=True,
         evaluate_each_gen=True,
         )
ee.result_of_experiment(search_all_gens=True)