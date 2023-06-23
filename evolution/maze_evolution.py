from src.maze_blimps import *
from evolution.evolutionBlimp import EvolutionExperiment





def expe_make(net, sim=None, port=23000, wakeup=None):
    ENTRY = (0, np.random.randint(0, 5))
    EXIT = (4, np.random.randint(0, 5))

    CENTER = np.array((1 + 2 * ENTRY[1], 5))
    R = 2.7

    def START_ZONE(i):
        out = CENTER - 2 * R
        while np.linalg.norm(out - CENTER) > R:
            out = CENTER + np.random.uniform(-R, R, 2)
        return (out[0], out[1], 1)

    def make_maze():
        H, W = (5, 5)
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
                orientation = 3 * np.pi / 2
            if cell[0] == H - 1 and not mm.initial_grid[cell[0]][cell[1]].walls['bottom']:
                wall = 'bottom'
                orientation = 0
            if cell[1] == W - 1 and not mm.initial_grid[cell[0]][cell[1]].walls['right']:
                wall = 'right'
                orientation = np.pi / 2
            orientations.append(orientation)
            walls.append(wall)
        return {'maze': mm,
                'entry': entry,
                'exit': ext,
                'entry_wall': walls[0],
                'entry_orientation': (0, 0, orientations[0]),
                'exit_wall': walls[1],
                'exit_orientation': (0, 0, orientations[1]), }

    return amazingBlimp(num_agents=5,
                        start_zone=START_ZONE,
                        scenePath=maze_view_path,
                        blimpPath=narrow_blimp_path,
                        cellPath=round_cell_path,
                        networkfn=net.activate,
                        end_time=10,
                        grid_size=2,
                        maze_entry_gen=make_maze,
                        wall_spawn_height=1,
                        wallPath=wall_path,
                        height_range=(1, 1),
                        use_ultra=True,
                        sim=sim,
                        simId=port,
                        wakeup=wakeup,
                        sleeptime=.01
                        )
ee=EvolutionExperiment('blimp_maze',expe_make)
ee.train(2,2,num_simulators=1,headless=False,restore=False)