from src.network_blimps import *
# from pymaze.src.maze_manager import MazeManager
from pymaze.src.maze import Maze

maze_view_path = os.path.join(DIR, 'scenes', 'maze_view.ttt')
wall_path = os.path.join(DIR, 'models', '2x2m_wall.ttm')
cell_path = os.path.join(DIR, 'models', 'cell_of_holding.ttm')
round_cell_path = os.path.join(DIR, 'models', 'round_cell_of_holding.ttm')


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
                 end_time,
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
                    'entry_orientation': orientation of entry
                    'entry_wall': wall to enter maze in (should be redundant)
                    'exit': location of exit
                non required keys, put in if you want a cell at the finish
                    'exit_orientation': orientation of entry
                    'exit_wall': wall to enter maze in (should be redundant)
        @param wallPath: path to the wall object to build the maze out of
        @param cellPath: path for cell of holding
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
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
        self.end_time = end_time
        self.cell_handle = None
        self.exit_cell_handle = None
        self.walls_to_handle = None

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
        self.walls_to_handle = dict()
        h_shift = np.array((self.grid_size/2, 0., 0.))
        v_shift = np.array((0., self.grid_size/2, 0.))
        cell_loc = np.zeros(3)
        exit_cell_loc = np.zeros(3)
        for i in range(self.maze.num_rows):
            for j in range(self.maze.num_cols):
                # NOTE: negative since it seems coordinates are row, -column
                # like row major order
                loc = np.array((j*self.grid_size + self.grid_size/2,
                                -i*self.grid_size - self.grid_size/2,
                                self.wall_spawn_height))
                if (i, j) == maze_dict['entry']:
                    cell_loc = loc
                if (i, j) == maze_dict['exit']:
                    exit_cell_loc = loc

                if self.maze.initial_grid[i][j].walls['right']:
                    wall = (i, j, 'right')
                    if wall not in self.walls_to_handle:
                        hand = self.spawnBlimp(self.wallPath, lambda: loc + h_shift, 1, (0, 0, np.pi/2))
                        self.walls_to_handle[wall] = hand
                        self.walls_to_handle[self.wall_pair(wall)] = hand

                if self.maze.initial_grid[i][j].walls['top']:
                    wall = (i, j, 'top')
                    if wall not in self.walls_to_handle:
                        hand = self.spawnBlimp(self.wallPath, lambda: loc + v_shift, 1)
                        self.walls_to_handle[wall] = hand
                        self.walls_to_handle[self.wall_pair(wall)] = hand

                if self.maze.initial_grid[i][j].walls['left']:
                    wall = (i, j, 'left')
                    if wall not in self.walls_to_handle:
                        hand = self.spawnBlimp(self.wallPath, lambda: loc - h_shift, 1, (0, 0, np.pi/2))
                        self.walls_to_handle[wall] = hand
                        self.walls_to_handle[self.wall_pair(wall)] = hand

                if self.maze.initial_grid[i][j].walls['bottom']:
                    wall = (i, j, 'bottom')
                    if wall not in self.walls_to_handle:
                        hand = self.spawnBlimp(self.wallPath, lambda: loc - v_shift, 1)
                        self.walls_to_handle[wall] = hand
                        self.walls_to_handle[self.wall_pair(wall)] = hand

        if maze_dict['entry_wall'] == 'top':
            cell_loc += v_shift
        elif maze_dict['entry_wall'] == 'bottom':
            cell_loc -= v_shift
        elif maze_dict['entry_wall'] == 'left':
            cell_loc -= h_shift
        else:
            cell_loc += h_shift
        self.cell_handle = self.spawnBlimp(self.cellPath, lambda: cell_loc, 1, maze_dict['entry_orientation'])
        if 'exit_wall' in maze_dict:
            if maze_dict['exit_wall'] == 'top':
                exit_cell_loc += v_shift
            elif maze_dict['exit_wall'] == 'bottom':
                exit_cell_loc -= v_shift
            elif maze_dict['exit_wall'] == 'left':
                exit_cell_loc -= h_shift
            else:
                exit_cell_loc += h_shift
            self.exit_cell_handle = self.spawnBlimp(self.cellPath, lambda: exit_cell_loc, 1,
                                                    maze_dict['exit_orientation'])
        super().spawnThings()

    ####################################################################################################################
    # Agent functions
    ####################################################################################################################
    def get_xy_pos(self, agent_id, spin=True):
        """
        returns 2-d position of agent

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: R^2 numpy array
        @return: position of agent
        """
        s = self.get_state(agent_id, spin=spin)
        return np.array((s['x'], s['y']))

    def get_grid_loc(self, agent_id, spin=True):
        """
        returns position of agent on grid

        @param agent_id: agent id
        @param spin: whether to update agent before getting state
        @rtype: N^2
        @return: grid location of agent
        """
        x, y = self.get_xy_pos(agent_id=agent_id, spin=spin)
        return self.y_to_i(y), self.x_to_j(x)

    def wall_btwn_agents(self, id0, id1, spin=False):
        """
        returns if there is a wall between two agents

        @param id0: first agent id
        @param id1: second agent id
        @param spin: whether to spin sensors before checking walls between
        @return: boolean, whether there is a wall blocking the line segment between the agents
        """
        if spin:
            self.spin([id0, id1])
        pos1 = self.get_xy_pos(id0, spin=False)
        pos2 = self.get_xy_pos(id1, spin=False)
        return self.is_wall_btwn(pos1, pos2)

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def end_test(self):
        """
        Runs at the end of step to decide termination of experiment

        @return: boolean of whether the experiment is done
        """
        return self.sim.getSimulationTime() > self.end_time

    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: negative averate manhattan distance from end tile,
                agent counts as 0 if completed (in exit cell)
        """
        s = []
        for agent_id in self.agentData:
            i, j = self.get_grid_loc(agent_id)
            ex_1, ex_2 = self.maze.exit_coor
            manhattan = abs(i - ex_1) + abs(j - ex_2)
            if i >= self.maze.num_rows:
                manhattan = 0
            s.append(-manhattan)
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                raise Exception("ERROR DEBUG")
        return np.mean(s)

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    @staticmethod
    def wall_pair(wall):
        """
        returns the other name for wall
            i.e. (0, 0, 'left') yields (0, 1, 'right')

        @param wall: wall to check, (i, j, direction)
        @return: pair for wall
        """
        i, j, st = wall
        if st == 'top':
            return i - 1, j, 'bottom'
        elif st == 'bottom':
            return i + 1, j, 'top'
        elif st == 'left':
            return (i, j - 1, 'right')
        else:  # st == 'right'
            return (i, j + 1, 'left')

    def y_to_i(self, y):
        """
        given a y coordinate, returns the i on the grid
            assumes we start from 0 and go negative

        @param y: y
        @return: i
        """
        return int(np.floor(-y/self.grid_size))

    def x_to_j(self, x):
        """
        given a x coordinate, returns the j on the grid
            assumes we start from 0 and go positive

        @param x: x
        @return: j
        """
        return int(np.floor(x/self.grid_size))

    def is_wall_btwn(self, pos1, pos2):
        """
        returns if there are any walls between two positions

        @param pos0: R^2, x,y
        @param pos1: R^2, x,y
        @return: boolean of if any of the walls between pos0 and pos1 are in the placed walls
        """
        return any(wall in self.walls_to_handle for wall in self.all_walls_btwn(pos1, pos2))

    def all_walls_btwn(self, pos0, pos1):
        """
        generator, yields all possible walls between pos0 and pos1

        @note: ignores redundancies
            e.g. will chose one of (0, 0, 'right'), (0, 1, 'left')

        @param pos0: R^2, x,y
        @param pos1: R^2, x,y
        @return: iterable of (i, j, direction)
            i.e. (0, 0, 'top')
        """
        d = self.grid_size
        x0, y0 = min(pos0, pos1, key=lambda pos: pos[0])
        x1, y1 = max(pos0, pos1, key=lambda pos: pos[0])
        # assert x0 <= x1
        x = np.ceil(x0/d)*d
        while x < x1:
            # now x0<x<x1
            r = (x - x0)/(x1 - x0)
            y = y0 + r*(y1 - y0)
            j = int(round(x/d))
            i = self.y_to_i(y)
            yield (i, j, 'left')
            x += d
        x0, y0 = min(pos0, pos1, key=lambda pos: pos[1])
        x1, y1 = max(pos0, pos1, key=lambda pos: pos[1])
        # assert y0 <= y1
        y = np.ceil(y0/d)*d
        while y < y1:
            # now y0<y<y1
            r = (y - y0)/(y1 - y0)
            x = x0 + r*(x1 - x0)
            i = int(round(-y/d))
            j = self.x_to_j(x)
            yield (i, j, 'top')
            y += d


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
                 end_time,
                 rng=2,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        test of maze blimp, network function takes octant sensing and returns a direction

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
        @param end_time: time to end experiment
        @param rng: range to sense neighbors
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
            end_time=end_time,
            height_factor=height_factor,
            sim=sim,
            simId=simId,
            msg_queue=msg_queue,
            wakeup=wakeup,
            sleeptime=sleeptime,
            spawn_tries=spawn_tries)
        self.rng = rng

    ####################################################################################################################
    # agent functions
    ####################################################################################################################
    def agent_sight(self, id0, id1, spin=False):
        """
        returns if first agent can 'see' other agent
            must be both in range and not blocked by wall

        @param id0: first agent id
        @param id1: other agent id
        @param spin: whether to spin sensors before checking
        @return: boolean, if first agent can 'see' other agent
        """
        if spin:
            self.spin([id0, id1])
        pos0 = self.get_xy_pos(id0, spin=False)
        pos1 = self.get_xy_pos(id1, spin=False)
        if np.linalg.norm(pos1 - pos0) > self.rng:
            return False
        return not self.is_wall_btwn(pos0, pos1)

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        gets the network input for agent specified

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """

        k_tant = self.get_neighbors_2d_k_ant(agent_id,
                                             is_neigh=self.agent_sight,
                                             k=8,
                                             spin=True)
        return k_tant.reshape((-1, 1))


if __name__ == "__main__":
    ENTRY = (0, np.random.randint(0, 5))
    EXIT = (4, np.random.randint(0, 5))

    CENTER = np.array((1 + 2*ENTRY[1], 5))
    R = 2.7


    def START_ZONE(i):
        out = CENTER - 2*R
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
                'exit_wall': walls[1],
                'exit_orientation': (0, 0, orientations[1]), }


    bb = amazingBlimp(num_agents=5,
                      start_zone=START_ZONE,
                      scenePath=maze_view_path,
                      blimpPath=narrow_blimp_path,
                      networkfn=lambda x: (0, -1),
                      height_range=(1, 1),
                      use_ultra=False,
                      maze_entry_gen=make_maze,
                      wallPath=wall_path,
                      end_time=100,
                      cellPath=round_cell_path,
                      grid_size=2,
                      wakeup=[COPPELIA_WAKEUP],
                      wall_spawn_height=1)
    print(bb.experiments(2))
    bb.kill()
