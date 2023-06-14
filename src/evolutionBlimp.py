from src.swarm_expiriment import *
import threading

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
            s.append(pos[2])
            print(self.get_state(agent_id))
        return np.mean(s)


class octantBlimp(blimpNet):
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
            s.append(pos[2])
        return np.mean(s)


import neat, os, sys

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


def eval_genom(genome, config, port,dict_to_unlock,key='locked'):
    genome.fitness = .0
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    exp = xyzBlimp(num_agents=1,
                   start_zone=lambda i: (0, 0, 1),
                   scenePath=empty_path,
                   blimpPath=narrow_blimp_path,
                   #networkfn=net.activate,
                   networkfn=(lambda x:(0.,0.,1.) if port==23000 else (1.,0.,0.)),
                   simId=port,
                   sleeptime=.01)
    trials = 1
    goals = exp.experiments(trials=trials, end_time=lambda t: t > 10)
    genome.fitness += sum(goals) / trials
    dict_to_unlock[key]=False
    del exp


def eval_genomes(genomes,
                 config,
                 num_simulators=2,
                 open_coppelia=True,
                 headless=False,
                 port_step=2,
                 coppelia='/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh',
                 zmq_def_port=23000,
                 websocket_def_port=23050,
                 close_after=True,
                 sleeptime=.1,
                 ):
    processes = dict()
    if open_coppelia:
        for i in range(num_simulators):
            zmqport = zmq_def_port + port_step * i
            processes[zmqport]=dict()
            cmd = coppelia + (' -h' if headless else '') + \
                  ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step * i) + \
                  ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            processes[zmqport]['subproc'] = p
            processes[zmqport]['pid'] = p.pid
            processes[zmqport]['locked'] = False  # if the simulator is being used for something

    for genome_id, genome in genomes:
        # for each genome, assign a port, and create a thread
        # the ports should unlock as soon as an earlier thread is done with them
        port_assigned=None
        while port_assigned is None:
            for zmqport in processes:
                if not processes[zmqport]['locked'] and ('thread' not in processes[zmqport] or
                                                         not processes[zmqport]['thread'].is_alive()):
                    processes[zmqport]['locked']=True
                    th=threading.Thread(target=lambda:eval_genom(genome,config,zmqport,processes[zmqport]),
                                        #daemon=True
                                        )
                    th.start()
                    time.sleep(sleeptime)
                    port_assigned=zmqport
                    processes[zmqport]['thread']=th
                    break
    # now make sure all threads are done
    done=False
    while not done:
        done=True
        for zmqport in processes:
            if 'thread' in processes[zmqport] and processes[zmqport]['thread'].is_alive():
                done=False
        time.sleep(sleeptime)
    # maybe close the coppelia processes
    if close_after:
        for zmqport in processes:
            kill(processes[zmqport]['pid'])


p = neat.Population(config)
p.add_reporter(neat.StdOutReporter(True))
stats = neat.StatisticsReporter()
p.add_reporter(stats)
winner = p.run(eval_genomes, 1)
print('\nBest genome:\n{!s}'.format(winner))
#TODO: state sensing doesnt work on specifically the second trial in each simulator???
# it is publishing, checked on ros ctrl test
print('\nOutput:')
winner_net = neat.nn.FeedForwardNetwork.create(winner, config)
exp = xyzBlimp(num_agents=1,
               start_zone=lambda i: (0, 0, 1),
               scenePath=empty_path,
               blimpPath=narrow_blimp_path,
               networkfn=winner_net.activate,
               wakeup=['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh']
               )
trials = 2
goals = exp.experiments(trials=trials, end_time=lambda t: t > 10)
print(goals)
exp.kill()