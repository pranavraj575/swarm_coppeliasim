from src.network_blimps import *
import os
import threading
import neat
from evolution.better_pop import better_Population
import gzip, random, pickle

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CHECKPT_DIR = os.path.join(DIR, 'checkpoints')
CONFIG_DIR = os.path.join(DIR, 'config')


def kill(proc_pid):
    """
    kills a process and processes it spawned
    @param proc_pid: id to kill
    """
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        try:
            proc.kill()
        except:
            pass
    process.kill()


def restore_checkpoint(filename):
    """
    Resumes the simulation from a previous saved point.
    @param filename: filename to restore
    """
    with gzip.open(filename) as f:
        generation, config, population, species_set, rndstate = pickle.load(f)
        random.setstate(rndstate)
        return better_Population(config, (population, species_set, generation))


def MOST_RECENT(dir):
    """
    returns name of most recent saved file in dir (by naming conventions, assumes name ends wtih -#)
    @param dir: directory to check
    @return: filename
    """
    out = None
    if os.path.exists(dir):
        for fil in os.listdir(dir):
            if out is None:
                out = fil
            else:
                out = max(fil, out, key=lambda f: int(f[f.rindex('-') + 1:]))
    return out


class EvolutionExperiment:
    def __init__(self,
                 name,
                 exp_maker):
        """
        experiment to use NEAT evolutionary algorithm on a Experiment class (specifically a blimpNet)
        @param name: folder name of experiment, used for config file and for checkpoint directories
        @param exp_maker: (net,sim,port,wakeup) -> src.network_blimps.blimpNet
                creates an Experiment to run given the NEAT network, simulator, port, and wakeup script
        """
        self.checkpt_dir = os.path.join(CHECKPT_DIR, name)
        if not os.path.exists(self.checkpt_dir):
            os.makedirs(self.checkpt_dir)
        config_file = os.path.join(CONFIG_DIR, name)
        self.config = neat.Config(
            neat.DefaultGenome,
            neat.DefaultReproduction,
            neat.DefaultSpeciesSet,
            neat.DefaultStagnation,
            config_file)
        self.exp_maker = exp_maker
    def train(self,
              generations,
              TRIALS,
              num_simulators=8,
              open_coppelia=True,
              headless=True,
              checkpt_freq=1,
              zmq_def_port=23000,
              websocket_def_port=23050,
              close_after=True,
              sleeptime=.1,
              resttime=.1,
              restore=True):
        if restore and MOST_RECENT(self.checkpt_dir) is not None:
            print('RESTORING')
            p = restore_checkpoint(os.path.join(self.checkpt_dir, MOST_RECENT(self.checkpt_dir)))
        else:
            p = better_Population(self.config)
        p.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        p.add_reporter(stats)
        p.add_reporter(
            neat.checkpoint.Checkpointer(checkpt_freq,
                                         filename_prefix=os.path.join(self.checkpt_dir, 'neat-checkpoint-')))

        winner = p.run(lambda genomes, config: self.eval_genomes(genomes=genomes,
                                                                 config=config,
                                                                 TRIALS=TRIALS,
                                                                 num_simulators=num_simulators,
                                                                 open_coppelia=open_coppelia,
                                                                 headless=headless,
                                                                 port_step=2,
                                                                 zmq_def_port=zmq_def_port,
                                                                 websocket_def_port=websocket_def_port,
                                                                 close_after=close_after,
                                                                 sleeptime=sleeptime,
                                                                 resttime=resttime,
                                                                 ),
                       generations)
        return winner

    def eval_genom(self,
                   genome,
                   config,
                   port,
                   TRIALS,
                   dict_to_unlock,
                   key,
                   sim):
        genome.fitness = .0
        net = neat.nn.FeedForwardNetwork.create(genome, config)

        exp = self.exp_maker(net=net, sim=sim, port=port,wakeup=None)
        goals = exp.experiments(trials=TRIALS)
        genome.fitness += sum(goals) / TRIALS
        dict_to_unlock[key] = False
        del exp

    def eval_genomes(self,
                     genomes,
                     config,
                     TRIALS,
                     num_simulators,
                     open_coppelia,
                     headless,
                     port_step,
                     zmq_def_port,
                     websocket_def_port,
                     close_after,
                     sleeptime,
                     resttime,
                     ):
        while True:
            try:
                if not rclpy.ok():
                    rclpy.init()
                break
            except:
                time.sleep(sleeptime)

        processes = dict()
        # open coppeliasim instances on different ports
        for i in range(num_simulators):
            zmqport = zmq_def_port + port_step * i

            if open_coppelia:
                processes[zmqport] = dict()
                cmd = COPPELIA_WAKEUP + (' -h' if headless else '') + \
                      ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step * i) + \
                      ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
                processes[zmqport]['subproc'] = p
                processes[zmqport]['pid'] = p.pid

        # connect zmq to the different coppeliasim instances
        for zmqport in processes:
            # must be done when simulator is running
            from zmqRemoteApi import RemoteAPIClient
            client = RemoteAPIClient(port=zmqport)
            processes[zmqport]['client'] = client
            processes[zmqport]['sim'] = client.getObject('sim')
            processes[zmqport]['locked'] = False  # if the simulator is being used for something
        start_time = time.time()

        # evaluate the genomes
        for genome_id, genome in genomes:
            if genome.fitness is not None:
                continue
            # for each genome, assign a port, and create a thread
            # the ports should unlock as soon as an earlier thread is done with them
            port_assigned = None
            while port_assigned is None:
                for zmqport in processes:
                    if not processes[zmqport]['locked'] and ('thread' not in processes[zmqport] or
                                                             not processes[zmqport]['thread'].is_alive()):
                        processes[zmqport]['locked'] = True
                        th = threading.Thread(target=lambda: self.eval_genom(genome=genome,
                                                                             config=config,
                                                                             TRIALS=TRIALS,
                                                                             port=zmqport,
                                                                             dict_to_unlock=processes[zmqport],
                                                                             key='locked',
                                                                             sim=processes[zmqport]['sim']
                                                                             ),
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
                        rclpy.shutdown()
                    break
                except:
                    time.sleep(sleeptime)
        time.sleep(resttime)

    def result_of_experiment(self, trials=1, search_all_gens=False, display=True, start_coppelia=True):
        if start_coppelia:
            wakeup = ['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh' +
                      ('' if display else ' -h')]
        else:
            wakeup = []
        winner = None
        for name in (os.listdir(self.checkpt_dir) if search_all_gens else [MOST_RECENT(self.checkpt_dir)]):
            p = restore_checkpoint(os.path.join(self.checkpt_dir, name))
            gen_winner = max([p.population[g] for g in p.population], key=lambda genome: genome.fitness)
            winner = gen_winner if winner is None else max([winner, gen_winner], key=lambda genome: genome.fitness)
        winner_net = neat.nn.FeedForwardNetwork.create(winner, self.config)
        exp = self.exp_maker(net=winner_net, wakeup=wakeup)
        goals = exp.experiments(trials=trials)
        exp.kill()
        return goals


L = 3
K = 4
INPUT_DIM = L * K


def START_ZONE(i):
    return ((-2, 2), (-2, 2), (1, 3))


def expe_make(net, sim=None, port=23000, wakeup=None):
    return xy_zero_Blimp(num_agents=5,
                         start_zone=START_ZONE,
                         scenePath=empty_path,
                         blimpPath=narrow_blimp_path,
                         networkfn=net.activate,
                         height_range=(1, 1),
                         use_ultra=True,
                         sim=sim,
                         simId=port,
                         wakeup=wakeup,
                         sleeptime=.01
                         )
    return l_k_tant_clump_blimp(num_agents=5,
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

ee = EvolutionExperiment('xy_zero_test', expe_make)
ee.train(2, 2)
print('here')
ee.result_of_experiment()
