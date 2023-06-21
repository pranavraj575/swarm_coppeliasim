from src.network_blimps import *
from pymaze.src.maze_manager import MazeManager
from pymaze.src.maze import Maze

wall_path = os.path.join(DIR, 'models', '2x2m_wall.ttm')
cell_path = os.path.join(DIR, 'models', 'cell_of_holding.ttm')


class aMazeBlimp(xyBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wallPath,
                 cellPath,
                 grid_size,
                 wall_spawn_height,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        An experiment where blimps spawn in a maze

        @note: since this inherits xyBlimp, output should be 2d

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param maze_entry_gen: () -> dict, generates a dictionary, required keys are
                'maze': pymaze.src.Maze,
                'entry': location of entry
                'orientation': orientation of entry
                'wall': wall to enter maze in (should be redundant)
                'exit': location of exit
        @param wallPath: path to the wall object to build the maze out of
        @param cellPath: path for cell of holding
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param height_factor: factor to multiply height adjust by
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            num_agents=num_agents,
            start_zone=start_zone,
            scenePath=scenePath,
            blimpPath=blimpPath,
            networkfn=networkfn,
            height_range=height_range,
            use_ultra=use_ultra,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.maze_entry_gen = maze_entry_gen
        self.maze = None
        self.wallPath = wallPath
        self.cellPath = cellPath
        self.grid_size = grid_size
        self.wall_handles = []
        self.wall_spawn_height = wall_spawn_height

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawnThings(self):
        """
        to be run at start of each expiriment
        """
        maze_dict = self.maze_entry_gen()
        self.maze = maze_dict['maze']
        # manager = MazeManager()
        # manager.add_existing_maze(self.maze)
        # manager.show_maze(self.maze.id)
        self.wall_handles = []
        h_shift = np.array((self.grid_size / 2, 0., 0.))
        v_shift = np.array((0., self.grid_size / 2, 0.))
        cell_loc = np.zeros(3)
        done = set()
        for i in range(self.maze.num_rows):
            for j in range(self.maze.num_cols):
                # NOTE: negative since it seems coordinates are row, -column
                # like row major order
                loc = np.array((j * self.grid_size + self.grid_size / 2,
                                -i * self.grid_size + self.grid_size / 2,
                                self.wall_spawn_height))
                if (i, j) == maze_dict['entry']:
                    cell_loc = loc

                if self.maze.initial_grid[i][j].walls['right']:
                    if (i, j, 'right') not in done:
                        self.wall_handles.append(
                            self.spawnBlimp(self.wallPath, lambda: loc + h_shift, 1, (0, 0, np.pi / 2)))
                    done.add((i, j, 'right'))
                    done.add((i, j + 1, 'left'))

                if self.maze.initial_grid[i][j].walls['top']:
                    if (i, j, 'top') not in done:
                        self.wall_handles.append(self.spawnBlimp(self.wallPath, lambda: loc + v_shift, 1))
                    done.add((i, j, 'top'))
                    done.add((i - 1, j, 'bottom'))

                if self.maze.initial_grid[i][j].walls['left']:
                    if (i, j, 'left') not in done:
                        self.wall_handles.append(
                            self.spawnBlimp(self.wallPath, lambda: loc - h_shift, 1, (0, 0, np.pi / 2)))
                    done.add((i, j, 'left'))
                    done.add((i, j - 1, 'right'))

                if self.maze.initial_grid[i][j].walls['bottom']:
                    if (i, j, 'bottom') not in done:
                        self.wall_handles.append(self.spawnBlimp(self.wallPath, lambda: loc - v_shift, 1))
                    done.add((i, j, 'bottom'))
                    done.add((i + 1, j, 'top'))

        if maze_dict['wall'] == 'top':
            cell_loc += v_shift
        elif maze_dict['wall'] == 'bottom':
            cell_loc -= v_shift
        elif maze_dict['wall'] == 'left':
            cell_loc -= h_shift
        else:
            cell_loc += h_shift
        self.cell_handle = self.spawnBlimp(self.cellPath, lambda: cell_loc, 1, maze_dict['orientation'])
        super().spawnThings()


class amazingBlimp(aMazeBlimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wallPath,
                 cellPath,
                 grid_size,
                 wall_spawn_height,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        test of maze blimp, network function ignores input and returns a direction

        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param maze_entry_gen: () -> dict, generates a dictionary, required keys are
                'maze': pymaze.src.Maze,
                'entry': location of entry
                'orientation': orientation of entry
                'wall': wall to enter maze in (should be redundant)
                'exit': location of exit
        @param wallPath: path to the wall object to build the maze out of
        @param cellPath: path for cell of holding
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param height_factor: factor to multiply height adjust by
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(
            num_agents=num_agents,
            start_zone=start_zone,
            scenePath=scenePath,
            blimpPath=blimpPath,
            networkfn=networkfn,
            height_range=height_range,
            use_ultra=use_ultra,
            maze_entry_gen=maze_entry_gen,
            wallPath=wallPath,
            cellPath=cellPath,
            grid_size=grid_size,
            wall_spawn_height=wall_spawn_height,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified
            since this is a test, just return None

        @param agent_id: agent to get input for
        @return: np array with correct dimensions
        """
        return None

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return False


ENTRY = (0, 2)


def START_ZONE(i):
    return ((2, 8), (4, 10), 1)


def make_maze():
    H, W = (5, 5)
    mm = Maze(H, W, entry=ENTRY, exit=(4, 2))
    entry = mm.entry_coor
    orientation = None
    wall = None
    if entry[0] == 0 and not mm.initial_grid[entry[0]][entry[1]].walls['top']:
        wall = 'top'
        orientation = np.pi
    if entry[1] == 0 and not mm.initial_grid[entry[0]][entry[1]].walls['left']:
        wall = 'left'
        orientation = 3 * np.pi / 2
    if entry[0] == H - 1 and not mm.initial_grid[entry[0]][entry[1]].walls['bottom']:
        wall = 'bottom'
        orientation = 0
    if entry[1] == W - 1 and not mm.initial_grid[entry[0]][entry[1]].walls['right']:
        wall = 'right'
        orientation = np.pi / 2
    return {'maze': mm,
            'entry': entry,
            'exit': mm.exit_coor,
            'wall': wall,
            'orientation': (0, 0, orientation)}


bb = amazingBlimp(num_agents=1,
                  start_zone=START_ZONE,
                  scenePath=empty_path,
                  blimpPath=narrow_blimp_path,
                  networkfn=lambda x: (0, -1),
                  height_range=(1, 1),
                  use_ultra=False,
                  maze_entry_gen=make_maze,
                  wallPath=wall_path,
                  cellPath=cell_path,
                  grid_size=2,
                  wakeup=[COPPELIA_WAKEUP],
                  wall_spawn_height=1)
bb.experiments(1)
