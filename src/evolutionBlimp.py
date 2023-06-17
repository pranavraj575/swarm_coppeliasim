from src.swarm_expiriment import *
import threading
import neat, os, sys


class blimpNet(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=1., ):
        """
        @param num_agents: number of blimps in this swarm expiriment
        @param start_zone: int -> (RxR U R)^3 goes from the blimp number to the spawn area
                (each dimension could be (value) or (low, high), chosen uniformly at random)
        @param scenePath: path to coppeliasim scene
        @param blimpPath: path to blimp for spawning
        @param networkfn: neural network function call for blimp to act
        @param sim: simulator, if already defined
        @param simId: simulator id, used to pass messages to correct topics
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        """
        super().__init__(num_agents,
                         start_zone,
                         scenePath,
                         blimpPath,
                         sim=sim,
                         simId=simId,
                         wakeup=wakeup,
                         sleeptime=sleeptime)
        self.network = networkfn

    ####################################################################################################################
    # network functions
    ####################################################################################################################
    def get_network_input(self, agent_id):
        """
        get network input for agent specified
        @return: numpy array of correct length/dimensions
        """
        raise NotImplementedError()

    def get_vec_from_net_ouput(self, output):
        """
        given network output, transform into control vector
            useful if we want edited controls (i.e. max speed or more drastic stuff like using position waypoints)
        @param output: network output
        @return: control vector, probably R^3 encoding velocity goal
        """
        return np.array(output).flatten()

    ####################################################################################################################
    # Expiriment functions (needs implementation in subclass)
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)
        """
        self.spin()
        for agent_id in self.agentData:
            z = self.network(self.get_network_input(agent_id))
            vec = self.get_vec_from_net_ouput(z)
            self.move_agent(agent_id, vec)


class xyzBlimp(blimpNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=1.):
        super().__init__(num_agents,
                         start_zone,
                         scenePath,
                         blimpPath,
                         networkfn,
                         sim=sim,
                         simId=simId,
                         wakeup=wakeup,
                         sleeptime=sleeptime)

    def get_network_input(self, agent_id):
        pos = self.get_position(agent_id, use_ultra=True, spin=True)
        return pos.reshape((3, 1))

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        returns z position right now
        """
        s = []
        for agent_id in self.agentData:
            pos = self.get_position(agent_id, use_ultra=False)
            # s.append(pos[2])
            s.append(-np.linalg.norm(pos))
            bug = self.get_state(agent_id)["DEBUG"]
            if bug == 0.:
                raise Exception("ERROR DEBUG")
        return np.mean(s)


class l_k_tantBlimp(blimpNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 l=3,
                 k=4,
                 rng=2,
                 sim=None,
                 simId=23000,
                 wakeup=None,
                 sleeptime=1.):
        super().__init__(num_agents,
                         start_zone,
                         scenePath,
                         blimpPath,
                         networkfn,
                         sim=sim,
                         simId=simId,
                         wakeup=wakeup,
                         sleeptime=sleeptime)
        self.l = l
        self.k = k
        self.rng = rng  # note we can do better than this, as this allows agents to see through walls

    def get_network_input(self, agent_id):
        l_k_tant = self.get_neighbors_3d_l_k_ant(agent_id, rng=self.rng, k=self.k, l=self.l, spin=True)
        return l_k_tant.reshape((-1, 1))

    def goal_data(self):
        """
        data to return at the end of each experiment trial
        minimizes distance between agents
        """
        s = []
        ids = list(self.agentData.keys())
        for i, id1 in enumerate(ids):
            for id2 in ids[i + 1:]:
                pos1 = self.get_position(id1, use_ultra=False)
                pos2 = self.get_position(id2, use_ultra=False)
                s.append(-np.linalg.norm(pos1 - pos2))
                # negative, we want to minimize distance between
        return np.mean(s)


DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
config_file = os.path.join(DIR, 'config', 'test-config-feedforward')
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_file)


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


L = 3
K = 4
INPUT_DIM = L * K


def START_ZONE(i):
    return ((-2, 2), (-2, 2), (1, 3))


def exp_make(net, sim=None, port=23000, wakeup=None):
    return l_k_tantBlimp(num_agents=5,
                         start_zone=START_ZONE,
                         scenePath=empty_path,
                         blimpPath=narrow_blimp_path,
                         networkfn=net.activate,
                         sim=sim,
                         simId=port,
                         sleeptime=.01,
                         l=L,
                         k=K,
                         wakeup=wakeup)


TRIALS = 2
END = lambda t: t > 60
CHECKPT_DIR = os.path.join(DIR, 'checkpoints', 'octant_blimp')
if not os.path.exists(CHECKPT_DIR):
    os.makedirs(CHECKPT_DIR)


def eval_genom(genome, config, port, dict_to_unlock, key='locked', sim=None):
    genome.fitness = .0
    net = neat.nn.FeedForwardNetwork.create(genome, config)

    exp = exp_make(net, sim, port)
    goals = exp.experiments(trials=TRIALS, end_time=END)
    genome.fitness += sum(goals) / TRIALS
    dict_to_unlock[key] = False
    del exp
    # print(genome.fitness)


def eval_genomes(genomes,
                 config,
                 num_simulators=6,
                 open_coppelia=True,
                 # this will open a bunch of coppelia sims on startup, if this is false, it just assumes they are open
                 # in the correct ports
                 headless=True,
                 port_step=2,
                 coppelia='/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh',
                 zmq_def_port=23000,
                 websocket_def_port=23050,
                 close_after=True,
                 sleeptime=.1,
                 resttime=1,
                 ):
    while True:
        try:
            if not rclpy.ok():
                rclpy.init()
            break
        except:
            time.sleep(sleeptime)
    processes = dict()
    for i in range(num_simulators):
        zmqport = zmq_def_port + port_step * i

        if open_coppelia:
            processes[zmqport] = dict()
            cmd = coppelia + (' -h' if headless else '') + \
                  ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step * i) + \
                  ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            processes[zmqport]['subproc'] = p
            processes[zmqport]['pid'] = p.pid
    for zmqport in processes:
        # must be done when simulator is running
        from zmqRemoteApi import RemoteAPIClient
        client = RemoteAPIClient(port=zmqport)
        processes[zmqport]['client'] = client
        processes[zmqport]['sim'] = client.getObject('sim')
        processes[zmqport]['locked'] = False  # if the simulator is being used for something
    start_time = time.time()
    for genome_id, genome in genomes:
        # for each genome, assign a port, and create a thread
        # the ports should unlock as soon as an earlier thread is done with them
        port_assigned = None
        while port_assigned is None:
            for zmqport in processes:
                if not processes[zmqport]['locked'] and ('thread' not in processes[zmqport] or
                                                         not processes[zmqport]['thread'].is_alive()):
                    processes[zmqport]['locked'] = True
                    th = threading.Thread(target=lambda: eval_genom(genome=genome,
                                                                    config=config,
                                                                    port=zmqport,
                                                                    dict_to_unlock=processes[zmqport],
                                                                    sim=processes[zmqport]['sim']
                                                                    ),
                                          # daemon=True
                                          )
                    th.start()
                    time.sleep(sleeptime)
                    port_assigned = zmqport
                    processes[zmqport]['thread'] = th
                    break
    # now make sure all threads are done
    done = False
    while not done:
        done = True
        for zmqport in processes:
            if 'thread' in processes[zmqport] and processes[zmqport]['thread'].is_alive():
                done = False
        time.sleep(sleeptime)
    # maybe close the coppelia processes
    dt = time.time() - start_time
    if close_after:
        for zmqport in processes:
            kill(processes[zmqport]['pid'])

        while True:
            try:
                if rclpy.ok():
                    # this fixes stuff??
                    # TODO: looks like the best way is to verify it works for one generation, then fully reset everything each gen
                    rclpy.shutdown()
                break
            except:
                time.sleep(sleeptime)
    time.sleep(resttime)


if False:
    p = neat.Population(config)
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)
    p.add_reporter(neat.Checkpointer(1, filename_prefix=os.path.join(CHECKPT_DIR, 'neat-checkpoint-')))
    winner = p.run(eval_genomes, 20)
    print('\nBest genome:\n{!s}'.format(winner))
    print('\nOutput:')
    winner_net = neat.nn.FeedForwardNetwork.create(winner, config)
    exp = exp_make(winner_net,
                   wakeup=['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh'])
    input("ENTER TO START")
    trials = 2
    goals = exp.experiments(trials=trials, end_time=END)
    print(goals)
    exp.kill()
p = neat.Checkpointer.restore_checkpoint(os.path.join(CHECKPT_DIR, 'neat-checkpoint-18'))
print(p)
print(dir(p))
for g in (p.population.values()):
    print(g.fitness)
quit()
winner_net = p.best_genome

exp = exp_make(winner_net,
               wakeup=['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh'])
goals = exp.experiments(trials=2, end_time=END)
print(goals)
exp.kill()
