from src.network_blimps import *
import os
from multiprocessing import Pool
import neat
from evolution.better_pop import better_Population
import gzip, random, pickle

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CHECKPT_DIR = os.path.join(DIR, 'checkpoints')
CONFIG_DIR = os.path.join(DIR, 'config')


def eval_genom(exp_maker,
               genome,
               config,
               port,
               TRIALS,
               ):
    """
    function to evaluate a single genome

    @param exp_maker: exp_maker: (net,sim,port,wakeup) -> src.network_blimps.blimpNet
            creates an Experiment to run given the NEAT network, simulator, port, and wakeup script

    @param genome: genome to evaluate
    @param config: config to use
    @param port: coppeliasim port to connect to
    @param TRIALS: number of trials per genome
    @return: fitness of the genome, None if process failed

    @note: this is defined at top level so that pool processing works,
        this will always be called inside EvolutionExperiment in practice
    """
    net = neat.nn.FeedForwardNetwork.create(genome, config)

    exp: blimpNet = exp_maker(net=net, sim=None, port=port, wakeup=None)
    goals = exp.experiments(trials=TRIALS)
    if goals is None:
        # process failed
        fitness = None
    else:
        fitness = sum(goals)/TRIALS
    exp.close_zmq()
    del exp
    return fitness


class EvolutionExperiment:
    def __init__(self,
                 checkpt_dir,
                 exp_maker,
                 config_name=None):
        """
        experiment to use NEAT evolutionary algorithm on a Experiment class (specifically a blimpNet)

        @param checkpt_dir: folder name of experiment, used for checkpoint directories and maybe for config file
        @param exp_maker: (net,sim,port,wakeup) -> src.network_blimps.blimpNet
                creates an Experiment to run given the NEAT network, simulator, port, and wakeup script
        @param config_name: file name of config file, defaults to the 'name' param

        @note: the output of an experiment must be a real number type, since it is used as fitness
        """
        if config_name is None:
            config_name = checkpt_dir
        self.checkpt_dir = checkpt_dir
        if not os.path.exists(self.checkpt_dir):
            raise Exception("CHECKPOINT PATH DOES NOT EXIST: " + self.checkpt_dir)
        config_file = os.path.join(CONFIG_DIR, config_name)
        self.config = neat.Config(
            neat.DefaultGenome,
            neat.DefaultReproduction,
            neat.DefaultSpeciesSet,
            neat.DefaultStagnation,
            config_file)
        self.exp_maker = exp_maker
        self.just_restored = False
        self.bandits = None
        self.failure = None
        self.current_num_sims = None

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
              restore=True,
              num_sim_range=None):
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
        @param num_sim_range: range to test 'num sims' parameters, None if just use given
        @return: best genome
        """
        if num_sim_range:
            self.bandits = {k: [] for k in range(num_sim_range[0], num_sim_range[1])}
            self.failure = {k: False for k in self.bandits}
        self.current_num_sims = num_simulators
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
                                                                 open_coppelia=open_coppelia,
                                                                 headless=headless,
                                                                 port_step=port_step,
                                                                 zmq_def_port=zmq_def_port,
                                                                 websocket_def_port=websocket_def_port,
                                                                 close_after=close_after,
                                                                 sleeptime=sleeptime,
                                                                 resttime=resttime,
                                                                 num_sim_range=num_sim_range
                                                                 ),
                       generations)
        return winner

    def eval_genomes(self,
                     genomes,
                     config,
                     TRIALS,
                     evaluate_each_gen,
                     open_coppelia,
                     headless,
                     port_step,
                     zmq_def_port,
                     websocket_def_port,
                     close_after,
                     sleeptime,
                     resttime,
                     num_sim_range
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
        @param open_coppelia: whether to open coppelia at the start
        @param headless: whether to run coppelia in headless mode
        @param port_step: amount to increment ports for different coppelia instances
        @param zmq_def_port: default port for ZMQ api
        @param websocket_def_port: default port for websocket
        @param close_after: whether to close coppela after done
        @param sleeptime: amount to sleep after important commands
        @param resttime: amount to rest after done
        @param num_sim_range: range to test 'num sims' parameters, None if just use given
        @return: elapsed time
        """
        while True:
            try:
                if not rclpy.ok():
                    rclpy.init()
                break
            except:
                time.sleep(sleeptime)
        if num_sim_range:
            if self.bandits[self.current_num_sims]:
                mean = np.mean(self.bandits[self.current_num_sims])
                # if we actually have test data, continue, otherwise just use self.current_num_sims
                temp_nsims = self.current_num_sims
                for s_temp in (self.current_num_sims - 1, self.current_num_sims + 1):
                    # if the data is unsampled, choose this
                    if s_temp in self.bandits:
                        if not self.bandits[s_temp]:
                            temp_nsims = s_temp

                for s_temp in (self.current_num_sims - 1, self.current_num_sims + 1):
                    # if we can decrease our time, choose this instead
                    if s_temp in self.bandits:
                        if self.bandits[s_temp] and np.mean(self.bandits[s_temp]) < mean:
                            temp_nsims = s_temp
                self.current_num_sims = temp_nsims

            print('using ' + str(self.current_num_sims) + " simulators")
        print('opening coppelia instances')
        processes = dict()
        # open coppeliasim instances on different ports
        pool = Pool(processes=self.current_num_sims)
        for k in range(self.current_num_sims):
            zmqport = zmq_def_port + port_step*k

            if open_coppelia:
                processes[zmqport] = dict()
                cmd = COPPELIA_WAKEUP + (' -h' if headless else '') + \
                      ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step*k) + \
                      ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
                processes[zmqport]['subproc'] = p
                processes[zmqport]['pid'] = p.pid

        for zmqport in processes:
            processes[zmqport]['genome'] = None
            processes[zmqport]['pool_worker'] = None

        #print('connecting to zmq')
        # connect zmq to the different coppeliasim instances
        #for zmqport in processes:
        #    # must be done when simulator is running
        #    from zmqRemoteApi import RemoteAPIClient
        #    client = RemoteAPIClient(port=zmqport)
        #    processes[zmqport]['client'] = client
        #    processes[zmqport]['sim'] = client.getObject('sim')
        #    processes[zmqport]['genome'] = None
        #    processes[zmqport]['pool_worker'] = None
        start_time = time.time()
        print('starting evaluation')
        # evaluate the genomes
        failed = True
        tries = 1
        while failed:
            j = 0
            failed = False
            for genome_id, genome in genomes:
                j += 1
                print('evaluating genome ' + str(j) + '/' + str(config.pop_size), end='\r')
                if self.just_restored:
                    # if we just restored, we can skip evaluating this generation
                    continue
                if (not evaluate_each_gen) and (genome.fitness is not None):
                    # we can skip if we are not evaluating pre-evaluated genomes, and this genome is pre-evaluated
                    continue
                # for each genome, assign a port, and create a process
                # the processes will finish after running the experiment
                port_assigned = None
                while port_assigned is None:
                    # loop to collect results
                    for zmqport in processes:
                        if processes[zmqport]['pool_worker'] is not None and processes[zmqport]['pool_worker'].ready():
                            fitness = processes[zmqport]['pool_worker'].get()
                            processes[zmqport]['genome'].fitness = fitness
                            processes[zmqport]['genome'] = None
                            processes[zmqport]['pool_worker'] = None
                    # loop to start new processes
                    for zmqport in processes:
                        if processes[zmqport]['genome'] is None:
                            processes[zmqport]['genome'] = genome
                            processes[zmqport]['pool_worker'] = pool.apply_async(eval_genom,
                                                                                 args=
                                                                                 [
                                                                                     self.exp_maker,
                                                                                     genome,
                                                                                     config,
                                                                                     TRIALS,
                                                                                     zmqport
                                                                                 ]
                                                                                 )
                            port_assigned = zmqport
                            break
                    time.sleep(sleeptime)
            # now make sure all processes are done
            done = False
            while not done:
                done = True
                for zmqport in processes:
                    if processes[zmqport]['pool_worker'] is not None:
                        done = False
                        if processes[zmqport]['pool_worker'].ready():
                            fitness = processes[zmqport]['pool_worker'].get()
                            processes[zmqport]['genome'].fitness = fitness
                            processes[zmqport]['genome'] = None
                            processes[zmqport]['pool_worker'] = None
                time.sleep(sleeptime)

            for genome_id, genome in genomes:
                if genome.fitness is None:
                    failed = True
            if failed:
                tries += 1
                print()
                print("FAILED SOME GENOME, TRYING AGAIN, time number " + str(tries))
        dt = time.time() - start_time
        print()
        # maybe close the coppelia processes
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
        if num_sim_range and not self.just_restored and \
                (tries == 1 or self.failure[self.current_num_sims]):
            # update if perfect run or if we have already failed at this number
            self.bandits[self.current_num_sims].append(dt)
        if tries > 1:
            self.failure[self.current_num_sims] = True
        if num_sim_range:
            print('running mean, std:')
            for k in self.bandits:
                if self.bandits[k]:
                    print(str(k) + ':', np.mean(self.bandits[k]), np.std(self.bandits[k]))
        self.just_restored = False
        pool.close()
        return dt

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
        if self.MOST_RECENT(self.checkpt_dir) is None:
            raise Exception("DIRECTORY EMPTY: " + self.checkpt_dir)
        if start_coppelia:
            wakeup = [COPPELIA_WAKEUP + ('' if display else ' -h')]
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


    ee = EvolutionExperiment('xy_zero_test', expe_make)
    ee.train(2, 2, evaluate_each_gen=True)
    print('here')
    ee.result_of_experiment()
