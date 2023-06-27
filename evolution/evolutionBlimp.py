from src.network_blimps import *
import os
import threading
import neat
from evolution.better_pop import better_Population
import gzip, random, pickle

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CHECKPT_DIR = os.path.join(DIR, 'checkpoints')
CONFIG_DIR = os.path.join(DIR, 'config')


class EvolutionExperiment:
    def __init__(self,
                 name,
                 exp_maker,
                 config_name=None):
        """
        experiment to use NEAT evolutionary algorithm on a Experiment class (specifically a blimpNet)

        @param name: folder name of experiment, used for checkpoint directories and maybe for config file
        @param exp_maker: (net,sim,port,wakeup) -> src.network_blimps.blimpNet
                creates an Experiment to run given the NEAT network, simulator, port, and wakeup script
        @param config_name: file name of config file, defaults to the 'name' param

        @note: the output of an experiment must be a real number type, since it is used as fitness
        """
        if config_name is None:
            config_name = name
        self.checkpt_dir = os.path.join(CHECKPT_DIR, name)
        if not os.path.exists(self.checkpt_dir):
            os.makedirs(self.checkpt_dir)
        config_file = os.path.join(CONFIG_DIR, config_name)
        self.config = neat.Config(
            neat.DefaultGenome,
            neat.DefaultReproduction,
            neat.DefaultSpeciesSet,
            neat.DefaultStagnation,
            config_file)
        self.exp_maker = exp_maker
        self.just_restored = False

    ####################################################################################################################
    # evolutionary training functions
    ####################################################################################################################
    def train(self,
              generations,
              TRIALS,
              evaluate_each_gen,
              num_simulators=8,
              open_coppelia=True,
              headless=True,
              checkpt_freq=1,
              port_step=2,
              zmq_def_port=23000,
              websocket_def_port=23050,
              close_after=True,
              sleeptime=.1,
              resttime=.1,
              restore=True):
        """
        Trains the population for a number of generations

        @param generations: number of generations to train for (NOTE: this includes the saved generation)
        @param TRIALS: trials to evaluate each genome
        @param evaluate_each_gen: whether to evaluate each genome each generation
            if False, keeps the fitness score of a genome evaluated in the previous generation
            This parameter will not affect the restored checkpoint generation
        @param num_simulators: number of coppelia sims to use
        @param open_coppelia: whether to open the simulators each generation
        @param headless: whether to run coppelia in headless mode
        @param checkpt_freq: how often to save checkpoints
        @param port_step: amount to increment ports for different coppelia instances
        @param zmq_def_port: default port for ZMQ API
        @param websocket_def_port: default port for the websocket
        @param close_after: whether to close instances of coppelia after running this
        @param sleeptime: time to sleep after important commands
        @param resttime: time to sleep after each generation
        @param restore: whether to restore progress
        @return: best genome
        """
        if restore and self.MOST_RECENT(self.checkpt_dir) is not None:
            print('RESTORING')
            p = self.restore_checkpoint(os.path.join(self.checkpt_dir, self.MOST_RECENT(self.checkpt_dir)))
            self.just_restored = True

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
                                                                 evaluate_each_gen=evaluate_each_gen,
                                                                 num_simulators=num_simulators,
                                                                 open_coppelia=open_coppelia,
                                                                 headless=headless,
                                                                 port_step=port_step,
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
                   sim,
                   print_genum):
        """
        function to evaluate a single genome

        @param genome: genome to evaluate
        @param config: config to use
        @param port: coppeliasim port to connect to
        @param TRIALS: number of trials per genome
        @param dict_to_unlock: dictionary to unlock, to indicate that we are done with simulator
        @param key: key in dictionary to unlock
        @param sim: simulator to use
        @param print_genum: prints number if positive, nothing if negative
        """
        genome.fitness = .0
        net = neat.nn.FeedForwardNetwork.create(genome, config)

        exp: blimpNet = self.exp_maker(net=net, sim=sim, port=port, wakeup=None)
        goals = exp.experiments(trials=TRIALS)
        genome.fitness += sum(goals)/TRIALS
        dict_to_unlock[key] = False
        del exp
        if print_genum >= 0:
            print('finished genome:', print_genum, '/', config.pop_size, end='\r')

    def eval_genomes(self,
                     genomes,
                     config,
                     TRIALS,
                     evaluate_each_gen,
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
        """
        evaluates all genomes, requirements specified in the NEAT document

        @note: what will be run is "lambda genomes, config: self.eval_genomes(genomes, config, ...)

        @param genomes: genomes to evaluate
        @param config: config to use
        @param TRIALS: trials to evaluate each genome
        @param evaluate_each_gen: whether to evaluate each genome each generation
            if False, keeps the fitness score of a genome evaluated in the previous generation
            This parameter will not affect the restored checkpoint generation
        @param num_simulators: number of simulators to use
        @param open_coppelia: whether to open coppelia at the start
        @param headless: whether to run coppelia in headless mode
        @param port_step: amount to increment ports for different coppelia instances
        @param zmq_def_port: default port for ZMQ api
        @param websocket_def_port: default port for websocket
        @param close_after: whether to close coppela after done
        @param sleeptime: amount to sleep after important commands
        @param resttime: amount to rest after done
        """
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
            zmqport = zmq_def_port + port_step*i

            if open_coppelia:
                processes[zmqport] = dict()
                cmd = COPPELIA_WAKEUP + (' -h' if headless else '') + \
                      ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step*i) + \
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
        i = 0
        for genome_id, genome in genomes:
            i += 1
            if self.just_restored:
                # if we just restored, we can skip evaluating this generation
                continue
            if (not evaluate_each_gen) and (genome.fitness is not None):
                # we can skip if we are not evaluating pre-evaluated genomes, and this genome is pre-evaluated
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
                                                                             sim=processes[zmqport]['sim'],
                                                                             print_genum=(
                                                                                 i if zmqport == zmq_def_port else -1)
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
                self.kill(processes[zmqport]['pid'])

            while True:
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                    break
                except:
                    time.sleep(sleeptime)
        time.sleep(resttime)
        self.just_restored = False

    ####################################################################################################################
    # output functions
    ####################################################################################################################
    def result_of_experiment(self, trials=1, search_all_gens=False, display=True, start_coppelia=True):
        """
        runs an experiment with best genome found, returns results

        @param trials: number of trials to run
        @param search_all_gens: whether to search all checkpoints or just the most recent one
        @param display: whether to open coppelia GUI
        @param start_coppelia: whether to start coppelia at the beginning
        @return: result of src.Experiment.experiments
        """
        if start_coppelia:
            wakeup = ['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh' +
                      ('' if display else ' -h')]
        else:
            wakeup = []
        winner = None
        for name in (os.listdir(self.checkpt_dir) if search_all_gens else [self.MOST_RECENT(self.checkpt_dir)]):
            p = self.restore_checkpoint(os.path.join(self.checkpt_dir, name))
            gen_winner = max([p.population[g] for g in p.population], key=lambda genome: genome.fitness)
            winner = gen_winner if winner is None else max([winner, gen_winner], key=lambda genome: genome.fitness)
        winner_net = neat.nn.FeedForwardNetwork.create(winner, self.config)
        exp: blimpNet = self.exp_maker(net=winner_net, wakeup=wakeup)
        goals = exp.experiments(trials=trials)
        exp.kill()
        return goals

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    @staticmethod
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

    @staticmethod
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

    @staticmethod
    def restore_checkpoint(filename):
        """
        Resumes the simulation from a previous saved point.

        @param filename: filename to restore
        """
        with gzip.open(filename) as f:
            generation, config, population, species_set, rndstate = pickle.load(f)
            random.setstate(rndstate)
            return better_Population(config, (population, species_set, generation))


if __name__ == "__main__":
    L = 3
    K = 4
    INPUT_DIM = L*K


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
    ee.train(2, 2, evaluate_each_gen=True)
    print('here')
    ee.result_of_experiment()
