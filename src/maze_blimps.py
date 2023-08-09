from src.network_blimps import *
from pymaze.src.maze import Maze
from collections import defaultdict

maze_view_path = os.path.join(DIR, 'scenes', 'maze_view.ttt')
wall_path = os.path.join(DIR, 'models', 'walls', '3m')
cell_path = os.path.join(DIR, 'models', 'cell_of_holding.ttm')
round_cell_path = os.path.join(DIR, 'models', 'round_cell_of_holding.ttm')


class aMazeBlimp(xyBlimp):
    def __init__(self,
                 num_agents,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wall_dir,
                 grid_size,
                 wall_spawn_height,
                 end_time,
                 start_squares=4,
                 cover_dir=os.path.join(DIR, 'models', 'covers'),
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
        @param wall_dir: path to the wall directory (files should look like 2x3.ttm)
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
        @param start_squares: number of squares for the starting area
        @param cover_dir: directory for lid of maze, None if no lid
            files should look like '5x5.ttm'
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
            start_zone=None,  # we will make start zone according to the maze
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
        self.grid_size = grid_size
        self.wall_handles = []
        self.wall_spawn_height = wall_spawn_height
        self.end_time = end_time

        self.exit_cell_handle = None
        self.walls_to_handle = None
        self.wall_dir = wall_dir
        self.cover_dir = cover_dir
        self.start_squares = start_squares
        self.len_to_wall = dict()
        self.still_at_spawn = None
        self.distance_dict = defaultdict(lambda: None)
        self.largest_possible = None
        for fn in os.listdir(self.wall_dir):
            if 'x' in fn:
                self.len_to_wall[float(fn[:fn.index('x')])] = os.path.join(self.wall_dir, fn)

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def spawn_largest_wall(self, locs, orientation):
        """
        spawns largest wall that covers the start of the locations
            assumes locs are consecutive and in order

        @param locs: list of locations, averaged to get center of wall
        @return: list of wall handles, locations used, locations unused
        """
        length = (len(locs)*self.grid_size)
        if length == int(length):
            length = int(length)
        else:
            print("WARNING, LENGTH IS NOT INTEGER")

        if length in self.len_to_wall:
            return (self.spawnModel(modelPath=self.len_to_wall[length],
                                    pos_rng=lambda: np.sum(locs, axis=0)/len(locs),
                                    orient_rng=lambda: orientation,
                                    spawn_tries=1),
                    locs,
                    [])
        if len(locs) == 1:
            raise Exception('wall of length ' + str(self.grid_size) + ' not in directory')
        handle, used, unused = self.spawn_largest_wall(locs[:-1], orientation)
        unused.append(locs[-1])
        return handle, used, unused

    def populate_distance_dict(self):
        """
        populates distance dict with values
        self.distance_dict[(i,j)] will return maze distance to exist square
        """
        if self.maze is None:
            raise Exception("this method should be called after making a maze")
        self.distance_dict[self.maze.exit_coor] = 0
        while not self.distance_dict_populated():
            to_add = defaultdict(lambda: float('inf'))
            for i in range(self.maze.num_rows):
                for j in range(self.maze.num_cols):
                    if (i, j) not in self.distance_dict:
                        for (wall_name, neighbor_cell) in (
                                ("top", (i - 1, j)),
                                ("bottom", (i + 1, j)),
                                ("left", (i, j - 1)),
                                ("right", (i, j + 1))):
                            if (not self.maze.initial_grid[i][j].walls[wall_name] and
                                    neighbor_cell in self.distance_dict):
                                to_add[(i, j)] = min(to_add[(i, j)], self.distance_dict[neighbor_cell] + 1)

            for cell in to_add:
                self.distance_dict[cell] = to_add[cell]
        self.largest_possible = 0
        for cell in self.distance_dict:
            self.largest_possible = max(self.largest_possible, self.distance_dict[cell])

    def distance_dict_populated(self):
        """
        boolean for if the distance dict is populated

        @return: True if all squares on maze have distance values
        """
        if self.maze is None:
            raise Exception("this method should be called after making a maze")
        for i in range(self.maze.num_rows):
            for j in range(self.maze.num_cols):
                if (i, j) not in self.distance_dict:
                    return False
        return True

    def spawn_best_wall(self, locs, orientation):
        """
        spawns walls of size self.grid_size that cover the locations
            assumes locs are consecutive and in order, spawns larger wall if possible

        @param locs: list of locations, averaged to get center of wall
        @return: list of (wall handle, locations)
        """
        hands = []
        while len(locs):
            hand, used, locs = self.spawn_largest_wall(locs, orientation=orientation)
            hands.append((hand, used))
        return hands

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
        self.maze_exit = maze_dict['exit']
        self.maze_entry = maze_dict['entry']
        h_shift = np.array((self.grid_size/2, 0., 0.))
        v_shift = np.array((0., self.grid_size/2, 0.))
        cell_loc = np.array((maze_dict['entry'][1]*self.grid_size + self.grid_size/2,
                             -maze_dict['entry'][0]*self.grid_size - self.grid_size/2,
                             self.wall_spawn_height))
        exit_cell_loc = np.array((maze_dict['exit'][1]*self.grid_size + self.grid_size/2,
                                  -maze_dict['exit'][0]*self.grid_size - self.grid_size/2,
                                  self.wall_spawn_height))

        for i in range(self.maze.num_rows + 1):  # also do last row
            locs = []
            walls = []
            for j in range(self.maze.num_cols):
                wall = (i, j, 'top')
                if i == self.maze.num_rows:
                    wall = self.wall_pair(wall)

                # NOTE: negative since it seems coordinates are row, -column
                # like row major order
                loc = np.array((wall[1]*self.grid_size + self.grid_size/2,
                                -wall[0]*self.grid_size - self.grid_size/2,
                                self.wall_spawn_height))

                if self.maze.initial_grid[wall[0]][wall[1]].walls[wall[2]]:
                    fixed_loc = loc + v_shift*(1 if wall[2] == 'top' else -1)
                    locs.append(fixed_loc)
                    walls.append(wall)
                else:
                    stuff = self.spawn_best_wall(locs=locs, orientation=(0, 0, 0))
                    locs = []
                    for hand, used in stuff:
                        for w in walls[:len(used)]:
                            self.walls_to_handle[w] = hand
                            self.walls_to_handle[self.wall_pair(w)] = hand
                        walls = walls[len(used):]

            stuff = self.spawn_best_wall(locs=locs, orientation=(0, 0, 0))
            for hand, used in stuff:
                for w in walls[:len(used)]:
                    self.walls_to_handle[w] = hand
                    self.walls_to_handle[self.wall_pair(w)] = hand
                walls = walls[len(used):]

        for j in range(self.maze.num_cols + 1):  # also do last col
            locs = []
            walls = []
            for i in range(self.maze.num_rows):
                wall = (i, j, 'left')
                if j == self.maze.num_cols:
                    wall = self.wall_pair(wall)

                loc = np.array((wall[1]*self.grid_size + self.grid_size/2,
                                -wall[0]*self.grid_size - self.grid_size/2,
                                self.wall_spawn_height))

                if self.maze.initial_grid[wall[0]][wall[1]].walls[wall[2]]:
                    fixed_loc = loc + h_shift*(1 if wall[2] == 'right' else -1)
                    locs.append(fixed_loc)
                    walls.append(wall)
                else:
                    stuff = self.spawn_best_wall(locs=locs, orientation=(0, 0, np.pi/2))
                    locs = []
                    for hand, used in stuff:
                        for w in walls[:len(used)]:
                            self.walls_to_handle[w] = hand
                            self.walls_to_handle[self.wall_pair(w)] = hand
                        walls = walls[len(used):]

            stuff = self.spawn_best_wall(locs=locs, orientation=(0, 0, np.pi/2))
            for hand, used in stuff:
                for w in walls[:len(used)]:
                    self.walls_to_handle[w] = hand
                    self.walls_to_handle[self.wall_pair(w)] = hand
                walls = walls[len(used):]

        spawn_locs1 = []
        spawn_locs2 = []
        top_loc = cell_loc
        spawn_cell = cell_loc + 0.
        temp_i, temp_j = maze_dict['entry']

        xmin, xmax = cell_loc[0] - self.grid_size/2, cell_loc[0] + self.grid_size/2
        ymin, ymax = cell_loc[1] - self.grid_size/2, cell_loc[1] + self.grid_size/2

        spawn_cells = []

        for k in range(self.start_squares):
            if maze_dict['entry_wall'] == 'top':
                spawn_cell += 2*v_shift
                spawn_locs1.append(spawn_cell + h_shift)
                spawn_locs2.append(spawn_cell - h_shift)
                top_loc = spawn_cell + v_shift
                ymax = top_loc[1]
                temp_i -= 1
            elif maze_dict['entry_wall'] == 'bottom':
                spawn_cell -= 2*v_shift
                spawn_locs1.append(spawn_cell + h_shift)
                spawn_locs2.append(spawn_cell - h_shift)
                top_loc = spawn_cell - v_shift
                ymin = top_loc[1]
                temp_i += 1
            elif maze_dict['entry_wall'] == 'left':
                spawn_cell -= 2*h_shift
                spawn_locs1.append(spawn_cell + v_shift)
                spawn_locs2.append(spawn_cell - v_shift)
                top_loc = spawn_cell - h_shift
                xmin = top_loc[0]
                temp_j -= 1
            else:
                spawn_cell += 2*h_shift
                spawn_locs1.append(spawn_cell + v_shift)
                spawn_locs2.append(spawn_cell - v_shift)
                top_loc = spawn_cell + h_shift
                xmax = top_loc[0]
                temp_j += 1
            self.spawn_hat(spawn_cell)
            spawn_cells.append((temp_i, temp_j))

        if maze_dict['entry_wall'] in ('top', 'bottom'):
            cap_orientation = (0, 0, 0)
            hall_orient = (0, 0, np.pi/2)
            hall_walls = ('left', 'right')
        else:
            cap_orientation = (0, 0, np.pi/2)
            hall_orient = (0, 0, 0)
            hall_walls = ('top', 'bottom')
        one = self.spawn_best_wall(locs=spawn_locs1, orientation=hall_orient)
        for i, j in spawn_cells:
            self.walls_to_handle[(i, j, hall_walls[0])] = one
            self.walls_to_handle[self.wall_pair((i, j, hall_walls[0]))] = one
        two = self.spawn_best_wall(locs=spawn_locs2, orientation=hall_orient)
        for i, j in spawn_cells:
            self.walls_to_handle[(i, j, hall_walls[1])] = two
            self.walls_to_handle[self.wall_pair((i, j, hall_walls[1]))] = two
        three = self.spawn_best_wall(locs=[top_loc], orientation=cap_orientation)
        self.walls_to_handle[(temp_i, temp_j, maze_dict['entry_wall'])] = three
        self.walls_to_handle[self.wall_pair((temp_i, temp_j, maze_dict['entry_wall']))] = three

        if maze_dict['entry_wall'] == 'top':
            cell_loc += v_shift
            self.still_at_spawn = lambda pos: pos[1] >= cell_loc[1]
        elif maze_dict['entry_wall'] == 'bottom':
            cell_loc -= v_shift
            self.still_at_spawn = lambda pos: pos[1] <= cell_loc[1]
        elif maze_dict['entry_wall'] == 'left':
            cell_loc -= h_shift
            self.still_at_spawn = lambda pos: pos[0] <= cell_loc[0]
        else:
            cell_loc += h_shift
            self.still_at_spawn = lambda pos: pos[0] >= cell_loc[0]

        def START(i):
            out = np.random.uniform((xmin, ymin, self.wall_spawn_height*.5), (xmax, ymax, self.wall_spawn_height*1.5))
            return out

        self.start_zone = START

        if 'exit_wall' in maze_dict:
            if maze_dict['exit_wall'] == 'top':
                exit_cell_loc += v_shift
            elif maze_dict['exit_wall'] == 'bottom':
                exit_cell_loc -= v_shift
            elif maze_dict['exit_wall'] == 'left':
                exit_cell_loc -= h_shift
            else:
                exit_cell_loc += h_shift

        if self.cover_dir is not None:
            h = self.maze.num_rows*self.grid_size
            if int(h) == h:
                h = int(h)
            w = self.maze.num_cols*self.grid_size
            if int(w) == w:
                w = int(w)
            fn = os.path.join(self.cover_dir, str(h) + 'x' + str(w) + '.ttm')
            if not os.path.exists(fn):
                print("ERROR: " + fn + ' does not exist, not spawning lid')
            else:
                self.lid_handle = self.spawnModel(modelPath=fn,
                                                  pos_rng=lambda: (self.grid_size*self.maze.num_cols/2,
                                                                   -self.grid_size*self.maze.num_rows/2,
                                                                   self.wall_spawn_height*2
                                                                   ),
                                                  spawn_tries=1)
        super().spawnThings()

    def spawn_hat(self, position):
        """
        spawns lid over a single position
        @param position: position to spawn lid

        @return: handle if lid spawned, None otherwise
        """
        if self.cover_dir is not None:
            d = self.grid_size
            if int(d) == d:
                d = int(d)

            fn = os.path.join(self.cover_dir, str(d) + 'x' + str(d) + '.ttm')
            if not os.path.exists(fn):
                print("ERROR: " + fn + ' does not exist, not spawning lid')
            else:
                return self.spawnModel(modelPath=fn,
                                       pos_rng=lambda: (position[0],
                                                        position[1],
                                                        self.wall_spawn_height*2
                                                        ),
                                       spawn_tries=1)
        return None

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
        for agent_id in self.agentData:
            if self.is_connected(agent_id):
                i, j = self.get_grid_loc(agent_id=agent_id, spin=False)
                pos = self.get_xy_pos(agent_id=agent_id, spin=False)
                if (i, j) == self.maze_exit:
                    return True

                if ((i >= self.maze.num_rows or i < 0) or
                        (j >= self.maze.num_cols or j < 0)):
                    if not self.still_at_spawn(pos):
                        return True
        return self.sim.getSimulationTime() > self.end_time

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

    def is_wall_btwn(self, pos0, pos1):
        """
        returns if there are any walls between two positions

        @param pos0: R^2, x,y
        @param pos1: R^2, x,y
        @return: boolean of if any of the walls between pos0 and pos1 are in the placed walls
        """
        return any(wall in self.walls_to_handle for wall in self.all_walls_btwn(pos0, pos1))

    def walls_of(self, cell):
        """
        returns walls in cell in order right, top, left, bottom
        @param cell: (i,j)
        @return [0,1]^4, 1 if wall, 0 if none
        """
        return np.array(
            [int((cell[0], cell[1], direct) in self.walls_to_handle) for direct in ('right', 'top', 'left', 'bottom')])

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
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wall_dir,
                 grid_size,
                 wall_spawn_height,
                 end_time,
                 start_squares,
                 cover_dir=os.path.join(DIR, 'models', 'covers'),
                 rng=2.,
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
        @param wall_dir: path to the wall directory (files should look like 2x3.ttm)
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
        @param start_squares: number of squares for the starting area
        @param cover_dir: directory for lid of maze, None if no lid
            files should look like '5x5.ttm'
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
            scenePath=scenePath,
            blimpPath=blimpPath,
            networkfn=networkfn,
            height_range=height_range,
            use_ultra=use_ultra,
            maze_entry_gen=maze_entry_gen,
            wall_dir=wall_dir,
            cover_dir=cover_dir,
            grid_size=grid_size,
            wall_spawn_height=wall_spawn_height,
            end_time=end_time,
            start_squares=start_squares,
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

        k_tant = self.get_neighbors_2d_k_tant(agent_id, is_neigh=self.agent_sight, k=8, spin=True)
        return k_tant.reshape((-1, 1))

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: negative averate manhattan distance from end tile,
                agent counts as 0 if completed (in exit cell)
        """
        s = []
        self.populate_distance_dict()
        for agent_id in self.agentData:
            i, j = self.get_grid_loc(agent_id)
            if (i, j) in self.distance_dict:
                s.append(-self.distance_dict[(i, j)])
            else:
                s.append(-self.largest_possible)
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None
        return np.mean(s)


class maxAmazingBlimp(amazingBlimp):
    def __init__(self,
                 num_agents,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wall_dir,
                 grid_size,
                 wall_spawn_height,
                 end_time,
                 start_squares,
                 cover_dir=os.path.join(DIR, 'models', 'covers'),
                 rng=2,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        maze blimp, fitness is recorded as the negative distance of the furthest blimp along
            if any blimps successfully complete the maze, fitness is the number of successful blimps

        @param num_agents: number of blimps in this swarm expiriment
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
        @param wall_dir: path to the wall directory (files should look like 2x3.ttm)
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
        @param start_squares: number of squares for the starting area
        @param cover_dir: directory for lid of maze, None if no lid
            files should look like '5x5.ttm'
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
        super().__init__(num_agents=num_agents,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         networkfn=networkfn,
                         height_range=height_range,
                         use_ultra=use_ultra,
                         maze_entry_gen=maze_entry_gen,
                         wall_dir=wall_dir,
                         grid_size=grid_size,
                         wall_spawn_height=wall_spawn_height,
                         end_time=end_time,
                         start_squares=start_squares,
                         cover_dir=cover_dir,
                         rng=rng,
                         height_factor=height_factor,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries)

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: negative averate manhattan distance from end tile,
                agent counts as 0 if completed (in exit cell)
        """
        self.populate_distance_dict()
        completed = 0
        furthest = -float('inf')
        for agent_id in self.agentData:
            i, j = self.get_grid_loc(agent_id)
            if (i, j) in self.distance_dict:
                dist = self.distance_dict[(i, j)]
            else:
                dist = self.largest_possible
            pos = self.get_xy_pos(agent_id)
            # if out of bounds and out of spawn area
            if ((i >= self.maze.num_rows or i < 0) or
                    (j >= self.maze.num_cols or j < 0)):
                if not self.still_at_spawn(pos):
                    dist = 0
            if dist <= 0:
                # finished the maze
                completed += 1
            furthest = max(furthest, -dist)
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                return None
        return furthest  # better to not worry about number completed
        return completed + furthest  # completed is positive iff furthest is 0


class ecosystemMaxAmazingBlimp(maxAmazingBlimp):
    def __init__(self,
                 num_agents,
                 scenePath,
                 blimpPath,
                 networkfns,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wall_dir,
                 grid_size,
                 wall_spawn_height,
                 end_time,
                 start_squares,
                 cover_dir=os.path.join(DIR, 'models', 'covers'),
                 rng=2,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        ecosystem implementation of max blimp
            this way is much easier than redefining everything as a ecosystemBlimpNet class
            the methods that need changing are goal_data and step

        @param num_agents: number of blimps in this swarm expiriment
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfns: agentid -> neural network function to call for blimp to act
            THIS ARG IS DIFFERENT from non-ecosystem class
        @param height_range: R^2, height range to keep blimps at
        @param use_ultra: whether to use ultrasound to set height (and as network input)
        @param maze_entry_gen: () -> dict, generates a dictionary, required keys are
                'maze': pymaze.src.Maze,
                'entry': location of entry
                'orientation': orientation of entry
                'wall': wall to enter maze in (should be redundant)
                'exit': location of exit
        @param wall_dir: path to the wall directory (files should look like 2x3.ttm)
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
        @param start_squares: number of squares for the starting area
        @param cover_dir: directory for lid of maze, None if no lid
            files should look like '5x5.ttm'
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
        super().__init__(num_agents=num_agents,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         networkfn=networkfns,
                         height_range=height_range,
                         use_ultra=use_ultra,
                         maze_entry_gen=maze_entry_gen,
                         wall_dir=wall_dir,
                         grid_size=grid_size,
                         wall_spawn_height=wall_spawn_height,
                         end_time=end_time,
                         start_squares=start_squares,
                         cover_dir=cover_dir,
                         rng=rng,
                         height_factor=height_factor,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries
                         )
        self.networks = self.network

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        self.spin()
        for agent_id in self.agentData:
            network = self.networks(agent_id)
            z = network(self.get_network_input(agent_id))
            vec = self.get_vec_from_net_ouput(z, agent_id)
            self.move_agent(agent_id, vec)
        t = self.sim.getSimulationTime()
        # print('cycle time:',t-self.last_time,end='\r')
        self.last_time = t
        return self.end_test()

    def goal_data(self):
        val = super().goal_data()
        return [val for _ in range(self.num_agents)]


class dist_sense_max_amazing_blimp(maxAmazingBlimp):
    def __init__(self,
                 num_agents,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wall_dir,
                 grid_size,
                 wall_spawn_height,
                 end_time,
                 start_squares,
                 cover_dir=os.path.join(DIR, 'models', 'covers'),
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        max blimp except blimps sense distance to neighbors instead of number of blimps nearby

        @param num_agents: number of blimps in this swarm expiriment
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
        @param wall_dir: path to the wall directory (files should look like 2x3.ttm)
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
        @param start_squares: number of squares for the starting area
        @param cover_dir: directory for lid of maze, None if no lid
            files should look like '5x5.ttm'
        @param height_factor: factor to multiply height adjust by
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(num_agents=num_agents,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         networkfn=networkfn,
                         height_range=height_range,
                         use_ultra=use_ultra,
                         maze_entry_gen=maze_entry_gen,
                         wall_dir=wall_dir,
                         grid_size=grid_size,
                         wall_spawn_height=wall_spawn_height,
                         end_time=end_time,
                         start_squares=start_squares,
                         cover_dir=cover_dir,
                         rng=float('inf'),
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

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """

        k_tant = self.global_get_inv_dist_2d_k_tant(agent_id, is_neigh=self.agent_sight, k=8, spin=True)
        return k_tant.reshape((-1, 1))


class wall_dist_sense_max_amazing_blimp(maxAmazingBlimp):
    def __init__(self,
                 num_agents,
                 scenePath,
                 blimpPath,
                 networkfn,
                 height_range,
                 use_ultra,
                 maze_entry_gen,
                 wall_dir,
                 grid_size,
                 wall_spawn_height,
                 end_time,
                 start_squares,
                 cover_dir=os.path.join(DIR, 'models', 'covers'),
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        max blimp except blimps sense distance to neighbors instead of number of blimps nearby

        @param num_agents: number of blimps in this swarm expiriment
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
        @param wall_dir: path to the wall directory (files should look like 2x3.ttm)
        @param grid_size: size of grid in m (should also match horizontal length of wall)
        @param wall_spawn_height: height to spawn wall
        @param end_time: time to end experiment
        @param start_squares: number of squares for the starting area
        @param cover_dir: directory for lid of maze, None if no lid
            files should look like '5x5.ttm'
        @param height_factor: factor to multiply height adjust by
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param msg_queue: queue length of ROS messages
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        @param spawn_tries: number of tries to spawn without collisions before giving up
                if 1, then sets position, does not change if collision detected
        """
        super().__init__(num_agents=num_agents,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         networkfn=networkfn,
                         height_range=height_range,
                         use_ultra=use_ultra,
                         maze_entry_gen=maze_entry_gen,
                         wall_dir=wall_dir,
                         grid_size=grid_size,
                         wall_spawn_height=wall_spawn_height,
                         end_time=end_time,
                         start_squares=start_squares,
                         cover_dir=cover_dir,
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

        @param agent_id: agent to get input for
        @return: R^(l*k) np array
        """

        k_tant = self.global_get_inv_dist_2d_k_tant(agent_id, is_neigh=self.agent_sight, k=8, spin=True)
        cell = self.get_grid_loc(agent_id=agent_id, spin=False)
        walls = self.walls_of(cell)
        return np.concatenate([k_tant, walls])


if __name__ == "__main__":
    H, W = (5, 5)
    EXIT = (0, np.random.randint(0, W))
    ENTRY = (H - 1, np.random.randint(0, W))
    D = 2
    CENTER = np.array((D/2 + D*ENTRY[1], 5))
    R = 2.7


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
                'exit_wall': walls[1],
                'exit_orientation': (0, 0, orientations[1]), }


    bb = wall_dist_sense_max_amazing_blimp(num_agents=20,
                                           scenePath=maze_view_path,
                                           blimpPath=narrow_blimp_path,
                                           networkfn=lambda x: (.5, 0),
                                           height_range=(1, 1),
                                           use_ultra=False,
                                           maze_entry_gen=make_maze,
                                           wall_dir=wall_path,
                                           end_time=20,
                                           start_squares=4,
                                           grid_size=D,
                                           wakeup=[COPPELIA_WAKEUP],
                                           wall_spawn_height=1.5,
                                           )
    print(bb.experiments(2))
    bb.kill()
