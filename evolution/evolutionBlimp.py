from src.network_blimps import *
import os
from multiprocessing import Pool
import neat
from evolution.better_pop import better_Population
import gzip, random, pickle
from collections import defaultdict

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))


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


def ecosystem_eval_genoms(ecosystem_exp_maker,
                          genomes,
                          config,
                          port,
                          TRIALS,
                          ):
    """
    function to evaluate genomes with multiple networks

    @param ecosystem_exp_maker: exp_maker: (nets,sim,port,wakeup) -> src.network_blimps.ecosystemBlimpNet
            creates an Experiment to run given the NEAT network, simulator, port, and wakeup script
            nets is of type (agentid -> network function)

    @param genomes: (agent id -> genome) or list of genomes, population of genomes to use
    @param config: config to use
    @param port: coppeliasim port to connect to
    @param TRIALS: number of trials per genome
    @return: fitness of the genome, None if process failed

    @note: this is defined at top level so that pool processing works,
        this will always be called inside EvolutionExperiment in practice
    """
    networks = [neat.nn.FeedForwardNetwork.create(genome, config) for genome in genomes]
    exp: blimpNet = ecosystem_exp_maker(nets=lambda i: networks[i], sim=None, port=port, wakeup=None)
    goals = exp.experiments(trials=TRIALS)
    if goals is None:
        # process failed
        fitness = None
    else:
        fitness = []
        for i in range(len(networks)):
            fitness.append(sum(goal[i] for goal in goals)/TRIALS)
    exp.close_zmq()
    del exp
    return fitness


class GeneralEvolutionaryExperiment:
    def __init__(self,
                 checkpt_dir,
                 exp_maker,
                 config_file):
        """
        experiment to use NEAT evolutionary algorithm on a Experiment class
            does not implement the evaluation of genomes, leaves that to subclasses
            functions need to be defined by subclasses

        @param checkpt_dir: folder name of experiment, used for checkpoint directories and maybe for config file
        @param exp_maker: (network arg,sim,port,wakeup) -> src.swarm_experiment.BlimpExperiment
                creates an Experiment to run given the NEAT network generation function, simulator, port, and wakeup script
                the network argument is different depending on subclass
        @param config_file: file path to config file

        @note: the output of an experiment must be a real number type, since it is used as fitness
        """
        self.checkpt_dir = checkpt_dir
        if not os.path.exists(self.checkpt_dir):
            raise Exception("CHECKPOINT PATH DOES NOT EXIST: " + self.checkpt_dir)
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
    # number of simulators update functions
    ####################################################################################################################
    def decide_num_sims(self, def_stdev=lambda val: abs(val)/2):
        """
        sets self.current_num_sims to best neighbor, based on sample of times using the mean and stdev found

        @param def_stdev: stdev to use for a given value if only one sample
        """
        if self.bandits[self.current_num_sims]:
            arr = self.bandits[self.current_num_sims]
            n = len(arr)
            mean = np.mean(arr)

            # sample standard deviation of mean
            stdev = np.std(arr)/np.sqrt(n - 1) if n > 1 else def_stdev(mean)
            sample = np.random.normal(mean, stdev)
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
                    yarr = self.bandits[s_temp]
                    rude = np.mean(yarr)
                    m = len(yarr)
                    stdev2 = np.std(yarr)/np.sqrt(m - 1) if m > 1 else def_stdev(rude)
                    if yarr and np.random.normal(rude, stdev2) < sample:
                        temp_nsims = s_temp
            self.current_num_sims = temp_nsims
        print('using ' + str(self.current_num_sims) + " simulators")

    def update_bandits(self, dt, tries):
        """
        updates the record of times for each num_sims in self.bandits
            assumes dt corresponds to self.current_num_sims

        @param dt: time elapsed
        @param tries: tries that it took to finish generation with no errors
        """
        if (self.bandits is not None) and not self.just_restored and \
                (tries == 1 or self.failure[self.current_num_sims]):
            # update if perfect run or if we have already failed at this number
            self.bandits[self.current_num_sims].append(dt)
        if tries > 1:
            self.failure[self.current_num_sims] = True
        if self.bandits is not None:
            print('running mean, std:')
            for k in self.bandits:
                if self.bandits[k]:
                    print(str(k) + ':', np.mean(self.bandits[k]), np.std(self.bandits[k]))

    ####################################################################################################################
    # evolutionary training functions
    ####################################################################################################################
    def train(self,
              generations,
              TRIALS,
              evaluate_each_gen,
              num_simulators=8,
              headless=True,
              checkpt_freq=1,
              port_step=2,
              zmq_def_port=23000,
              websocket_def_port=23050,
              sleeptime=.1,
              resttime=.1,
              restore=True,
              num_sim_range=None,
              reset_after_first_fail=True,
              debug=False
              ):
        """
        Trains the population for a number of generations

        @param generations: number of generations to train for (NOTE: this includes the saved generation)
        @param TRIALS: trials to evaluate each genome
        @param evaluate_each_gen: whether to evaluate each genome each generation
            if False, keeps the fitness score of a genome evaluated in the previous generation
            This parameter will not affect the restored checkpoint generation
        @param num_simulators: number of coppelia sims to use
        @param headless: whether to run coppelia in headless mode
        @param checkpt_freq: how often to save checkpoints
        @param port_step: amount to increment ports for different coppelia instances
        @param zmq_def_port: default port for ZMQ API
        @param websocket_def_port: default port for the websocket
        @param sleeptime: time to sleep after important commands
        @param resttime: time to sleep after each generation
        @param restore: whether to restore progress
        @param num_sim_range: range to test 'num sims' parameters, None if just use given
        @param reset_after_first_fail: if we reset all coppeliasims after one failure to connect
        @param debug: whether to print excessive debug messages
        @return: best genome
        """
        if num_sim_range:
            self.bandits = {k: [] for k in range(num_sim_range[0], num_sim_range[1])}
            self.failure = {k: False for k in self.bandits}
        else:
            self.bandits = {num_simulators: []}
            self.failure = {num_simulators: False}
        self.current_num_sims = num_simulators
        if restore and self.MOST_RECENT(self.checkpt_dir) is not None:
            print('RESTORING')
            p = self.restore_checkpoint(os.path.join(self.checkpt_dir, self.MOST_RECENT(self.checkpt_dir)[-1]))
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
                                                                 headless=headless,
                                                                 port_step=port_step,
                                                                 zmq_def_port=zmq_def_port,
                                                                 websocket_def_port=websocket_def_port,
                                                                 sleeptime=sleeptime,
                                                                 resttime=resttime,
                                                                 reset_after_first_fail=reset_after_first_fail,
                                                                 debug=debug
                                                                 ),
                       generations)
        return winner

    def eval_genomes(self,
                     genomes,
                     config,
                     TRIALS,
                     evaluate_each_gen,
                     headless,
                     port_step,
                     zmq_def_port,
                     websocket_def_port,
                     sleeptime,
                     resttime,
                     reset_after_first_fail,
                     debug,
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
        @param headless: whether to run coppelia in headless mode
        @param port_step: amount to increment ports for different coppelia instances
        @param zmq_def_port: default port for ZMQ api
        @param websocket_def_port: default port for websocket
        @param sleeptime: amount to sleep after important commands
        @param resttime: amount to rest after done
        @param reset_after_first_fail: if we reset all coppeliasims after one failure to connect
        @param debug: whether to print excessive debug messages
        @return: elapsed time
        """
        if self.bandits is not None:
            self.decide_num_sims()
        start_time = time.time()
        print('starting evaluation')
        # evaluate the genomes
        tries = self.inner_loop(genomes=genomes,
                                config=config,
                                TRIALS=TRIALS,
                                evaluate_each_gen=evaluate_each_gen,
                                sleeptime=sleeptime,
                                reset_after_first_fail=reset_after_first_fail,
                                zmq_def_port=zmq_def_port,
                                websocket_def_port=websocket_def_port,
                                port_step=port_step,
                                headless=headless,
                                debug=debug)
        dt = time.time() - start_time
        time.sleep(resttime)
        self.update_bandits(dt, tries)
        self.just_restored = False

        return dt

    def close_coppelia_sims(self):
        for zmqport in self.processes:
            self.kill(self.processes[zmqport]['pid'])

    def make_coppelia_sims(self, zmq_def_port, websocket_def_port, port_step, headless):
        """
        makes self.current_num_sims instances of coppeliasim

        @param zmq_def_port: default port for zmq api
        @param websocket_def_port: default port for websocket api
        @param port_step: amount to change ports (should be 2 probably)
        @param headless: whether to run headless
        """
        print('opening coppelia sims')
        self.processes = defaultdict(lambda: defaultdict(lambda: None))
        # open coppeliasim instances on different ports
        for k in range(self.current_num_sims):
            zmqport = zmq_def_port + port_step*k
            cmd = COPPELIA_WAKEUP + (' -h' if headless else '') + \
                  ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step*k) + \
                  ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            self.processes[zmqport]['subproc'] = p
            self.processes[zmqport]['pid'] = p.pid

    def activate_rclpy(self, sleeptime):
        """
        activates rclpy

        @param sleeptime: time to rest between attempts
        """
        while True:
            try:
                if not rclpy.ok():
                    rclpy.init()
                break
            except:
                time.sleep(sleeptime)

    def shutdown_rclpy(self, sleeptime):
        """
        deactivates rclpy

        @param sleeptime: time to rest between attempts
        """
        while True:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
                break
            except:
                time.sleep(sleeptime)

    def processes_active(self):
        """
        returns list of processes active

        @return: list of genome orders
        """
        out = []
        for zmqport in self.processes:
            if self.processes[zmqport]['pool_worker'] is not None:
                out.append(self.processes[zmqport]['genome order'])
        return out

    def collect_genome_fitnesses(self, debug):
        """
        loops through processes and saves the fitnesses of genomes if they are done

        @param debug: whether to print excessive debug messages
        @return: whether the processes are all done
        """
        raise NotImplementedError()

    def inner_loop(self, genomes,
                   config,
                   TRIALS,
                   evaluate_each_gen,
                   sleeptime,
                   reset_after_first_fail,
                   zmq_def_port,
                   websocket_def_port,
                   port_step,
                   headless,
                   debug):
        """
        looops through genomes, creates an experiment with that network, and sets the genome fitness

        @param genomes: genomes to evaluate
        @param config: config to use
        @param TRIALS: trials to evaluate each genome
        @param sleeptime:
        @param evaluate_each_gen: whether to evaluate each genome each generation
            if False, keeps the fitness score of a genome evaluated in the previous generation
            This parameter will not affect the restored checkpoint generation
        @param sleeptime: amount to sleep after important commands
        @param reset_after_first_fail: if we reset all coppeliasims after one failure to connect
        @param zmq_def_port: default port for ZMQ api
        @param websocket_def_port: default port for websocket
        @param port_step: amount to increment ports for different coppelia instances
        @param headless: whether to run coppelia in headless mode
        @param debug: whether to print excessive debug messages
        @return: number of tries it took to finish all genomes
            if more than 1, some error probably happened
        """
        raise NotImplementedError()

    ####################################################################################################################
    # output functions
    ####################################################################################################################
    def result_of_experiment(self, trials=1, gen_indices=(-1,), network_to_use=None, display=True, zmqport=23000,
                             websocket_port=23050):
        """
        runs an experiment with best genome found, returns results

        @param trials: number of trials to run
        @param gen_indices: which generations to show, defaults to just last one
        @param network_to_use: if defined, use this network to act in environment
            function with the same inputs and outputs of the neural network in the environment
            leave as None if we want to use the evolved network
        @param display: whether to open coppelia GUI
        @param zmqport: port to use for zmq
        @param websocket_port: port to use for websocket
        @return: result of src.Experiment.experiments
        """
        raise NotImplementedError()

    def show_stats(self):
        stats = self.get_stats()
        gens = list(stats.keys())
        gens.sort()
        for g in gens:
            print("GEN:", g)
            for field in stats[g]:
                if field == 'species':
                    for specid in stats[g]['species']:
                        print('\t species id:', specid)
                        spec_dict = stats[g]['species'][specid]
                        for specfield in spec_dict:
                            print('\t\t' + specfield + ':', spec_dict[specfield])
                else:
                    print(field + ':', stats[g][field])

    def get_stats(self):
        if self.MOST_RECENT(self.checkpt_dir) is None:
            raise Exception("DIRECTORY EMPTY: " + self.checkpt_dir)
        stats = dict()
        for direct in self.MOST_RECENT(self.checkpt_dir):
            stat_dict = dict()
            gen = int(direct[direct.rindex('-') + 1:])
            p = self.restore_checkpoint(os.path.join(self.checkpt_dir, direct))
            fitnesses = [p.population[g].fitness for g in p.population]
            stat_dict['best_fitness'] = max(fitnesses)
            stat_dict['mean_fitness'] = np.mean(fitnesses)
            stat_dict['stdev_fitness'] = np.std(fitnesses)
            stat_dict['min_fitness'] = min(fitnesses)

            specy = p.species.species
            stat_dict['species'] = dict()
            for specid in specy:
                specy_dict = dict()
                specy_dict['fitness'] = specy[specid].fitness
                specy_dict['adjusted_fitness'] = specy[specid].adjusted_fitness
                specy_dict['last_improved'] = specy[specid].last_improved
                specy_dict['members'] = len(specy[specid].members)
                specy_dict['created'] = specy[specid].created
                stat_dict['species'][specid] = specy_dict

            # winner = max([p.population[g] for g in p.population], key=lambda genome: genome.fitness)
            # winner_net = neat.nn.FeedForwardNetwork.create(winner, self.config)

            stats[gen] = stat_dict
        return stats

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
            out = [f for f in os.listdir(dir) if '-' in f]
            if not out:
                return None
            out.sort(key=lambda f: int(f[f.rindex('-') + 1:]))
        return out

    @staticmethod
    def kill_individual(proc_pid, sleep=.01):
        """
        kills a single process, waits till it is dead
        """
        process = psutil.Process(proc_pid)
        while process.status() not in (psutil.STATUS_ZOMBIE,
                                       psutil.STATUS_DEAD,
                                       psutil.STATUS_STOPPED):
            process.kill()
            time.sleep(sleep)

    def kill(self, proc_pid):
        """
        kills a process and processes it spawned

        @param proc_pid: id to kill
        """
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            try:
                self.kill_individual(proc.pid)
            except:
                pass
        try:
            self.kill_individual(process.pid)
        except:
            pass

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


class EvolutionExperiment(GeneralEvolutionaryExperiment):
    def __init__(self, checkpt_dir, exp_maker, config_file):
        """

        experiment to use NEAT evolutionary algorithm on a Experiment class (specifically a blimpNet)

        @param checkpt_dir: folder name of experiment, used for checkpoint directories and maybe for config file
        @param exp_maker: (net,sim,port,wakeup) -> src.network_blimps.blimpNet
                    creates an Experiment to run given the NEAT network, simulator, port, and wakeup script
        @param config_file: file path to config file

        @note: the output of an experiment must be a real number type, since it is used as fitness
        """
        super().__init__(checkpt_dir=checkpt_dir, exp_maker=exp_maker, config_file=config_file)

    ####################################################################################################################
    # evolutionary training functions
    ####################################################################################################################
    def collect_genome_fitnesses(self, debug):
        """
        loops through processes and saves the fitnesses of genomes if they are done

        @param debug: whether to print excessive debug messages
        @return: whether the processes are all done
        """
        done = True
        for zmqport in self.processes:
            if self.processes[zmqport]['pool_worker'] is not None:
                done = False
                if self.processes[zmqport]['pool_worker'].ready():
                    fitness = self.processes[zmqport]['pool_worker'].get()
                    if fitness is None:
                        self.failed_genomes.append(self.processes[zmqport]['genome'])
                    self.processes[zmqport]['genome'].fitness = fitness
                    self.processes[zmqport]['genome'] = None
                    self.processes[zmqport]['pool_worker'] = None
                    if fitness is None:  # do this anyway
                        print()
                        print('failed genome:', self.processes[zmqport]['genome order'])
                    if debug and fitness is not None:
                        print()
                        print('got genome:', self.processes[zmqport]['genome order'])

        return done

    def inner_loop(self,
                   genomes,
                   config,
                   TRIALS,
                   evaluate_each_gen,
                   sleeptime,
                   reset_after_first_fail,
                   zmq_def_port,
                   websocket_def_port,
                   port_step,
                   headless,
                   debug):
        """
        looops through genomes, creates an experiment with that network, and sets the genome fitness

        @param genomes: genomes to evaluate
        @param config: config to use
        @param TRIALS: trials to evaluate each genome
        @param sleeptime:
        @param evaluate_each_gen: whether to evaluate each genome each generation
            if False, keeps the fitness score of a genome evaluated in the previous generation
            This parameter will not affect the restored checkpoint generation
        @param sleeptime: amount to sleep after important commands
        @param reset_after_first_fail: if we reset all coppeliasims after one failure to connect
        @param zmq_def_port: default port for ZMQ api
        @param websocket_def_port: default port for websocket
        @param port_step: amount to increment ports for different coppelia instances
        @param headless: whether to run coppelia in headless mode
        @param debug: whether to print excessive debug messages
        @return: number of tries it took to finish all genomes
            if more than 1, some error probably happened
        """
        pool = Pool(processes=self.current_num_sims)
        self.failed_genomes = []
        tries = 0
        global_polulation = []
        for genome_id, genome in genomes:
            global_polulation.append(genome)
        while tries == 0 or self.failed_genomes:
            self.activate_rclpy(sleeptime=sleeptime)
            self.make_coppelia_sims(zmq_def_port=zmq_def_port,
                                    websocket_def_port=websocket_def_port,
                                    port_step=port_step,
                                    headless=headless)
            tries += 1
            j = 0
            if tries == 1:
                todo = global_polulation
            else:
                todo = tuple(self.failed_genomes)
                self.failed_genomes = []

            for genome in todo:
                j += 1
                skip = False
                if self.just_restored:
                    # if we just restored, we can skip evaluating this generation
                    skip = True
                if tries == 1:
                    # IF we only are on first try:
                    if (not evaluate_each_gen) and (genome.fitness is not None):
                        # we can skip if we are not evaluating pre-evaluated genomes, and this genome is pre-evaluated
                        skip = True
                else:
                    # otherwise, we already did this
                    if genome.fitness is not None:
                        skip = True

                if (not skip) and reset_after_first_fail and self.failed_genomes:
                    # if we have already failed, and we havent skipped already
                    self.failed_genomes.append(genome)
                    skip = True
                print(('skipping' if skip else 'evaluating') + ' genome ' + str(j) + '/' + str(len(todo)),
                      end=('\n' if debug else '\r'))
                if skip:
                    continue
                # for each genome, assign a port, and create a process
                # the processes will finish after running the experiment
                port_assigned = None
                while port_assigned is None:
                    # loop to collect results
                    self.collect_genome_fitnesses(debug=debug)

                    # loop to start new self.processes
                    for zmqport in self.processes:
                        if self.processes[zmqport]['genome'] is None:
                            self.processes[zmqport]['genome'] = genome
                            self.processes[zmqport]['genome order'] = j
                            self.processes[zmqport]['pool_worker'] = pool.apply_async(eval_genom,
                                                                                      args=
                                                                                      [
                                                                                          self.exp_maker,
                                                                                          genome,
                                                                                          config,
                                                                                          zmqport,
                                                                                          TRIALS,
                                                                                      ]
                                                                                      )
                            port_assigned = zmqport
                            break
                    time.sleep(sleeptime)
            # now make sure all processes are done
            while not self.collect_genome_fitnesses(debug=debug):
                # will run until all processes are done
                # i.e. when collect_genome_fitnesses returns True
                time.sleep(sleeptime)
                if debug:
                    print('left: ', self.processes_active(), '           ', end='\r')

            if self.failed_genomes:
                print()
                print("FAILED SOME GENOME, TRYING AGAIN, time number " + str(tries + 1))
            self.close_coppelia_sims()
            self.shutdown_rclpy(sleeptime=sleeptime)
        print()
        pool.close()
        return tries

    ####################################################################################################################
    # output functions
    ####################################################################################################################
    def result_of_experiment(self, trials=1, gen_indices=(-1,), network_to_use=None, display=True, zmqport=23000,
                             websocket_port=23050):
        """
        runs an experiment with best genome found, returns results

        @param trials: number of trials to run
        @param gen_indices: which generations to show, defaults to just last one
        @param network_to_use: if defined, use this network to act in environment
            function with the same inputs and outputs of the neural network in the environment
            leave as None if we want to use the evolved network
        @param display: whether to open coppelia GUI
        @param zmqport: port to use for zmq
        @param websocket_port: port to use for websocket
        @return: result of src.Experiment.experiments
        """
        if self.MOST_RECENT(self.checkpt_dir) is None:
            raise Exception("DIRECTORY EMPTY: " + self.checkpt_dir)

        cmd = COPPELIA_WAKEUP + ('' if display else ' -h') + \
              ' -GwsRemoteApi.port=' + str(websocket_port) + \
              ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
        wakeup = [cmd]
        all_goals = []
        for index in gen_indices:
            path = os.path.join(self.checkpt_dir, self.MOST_RECENT(self.checkpt_dir)[index])
            p = self.restore_checkpoint(path)
            print("DISPLAYING:", path)
            if network_to_use is None:
                winner = max([p.population[g] for g in p.population], key=lambda genome: genome.fitness)
                network = neat.nn.FeedForwardNetwork.create(winner, self.config)
            else:
                network = network_to_use
            exp: blimpNet = self.exp_maker(net=network, wakeup=wakeup, port=zmqport)
            goals = exp.experiments(trials=trials)
            exp.kill()

            del exp

            all_goals.append(goals)
        return all_goals


class EcosystemEvolutionExperiment(GeneralEvolutionaryExperiment):
    def __init__(self, checkpt_dir, ecosystem_exp_maker, num_agents, config_file):
        """

        experiment to use NEAT evolutionary algorithm on a Experiment class (specifically a ecosystemBlimpNet)

        @param checkpt_dir: folder name of experiment, used for checkpoint directories and maybe for config file
        @param ecosystem_exp_maker: (nets,sim,port,wakeup) -> src.network_blimps.ecosystemBlimpNet
            creates an Experiment to run given the NEAT network, simulator, port, and wakeup script
            nets is of type (agentid -> network function)
        @param num_agents: number of agents to use in an environment
        @param config_file: file path to config file

        @note: the output of an experiment must be a list of real numbers, in order of each agent in the environment
        """
        super().__init__(checkpt_dir=checkpt_dir, exp_maker=ecosystem_exp_maker, config_file=config_file)
        self.num_agents = num_agents
        self.failed_genomes = None

    ####################################################################################################################
    # evolutionary training functions
    ####################################################################################################################
    def collect_genome_fitnesses(self, debug):
        """
        loops through processes and saves the fitnesses of genomes if they are done

        @param debug: whether to print excessive debug messages
        @return: whether the processes are all done
        """
        done = True
        for zmqport in self.processes:
            if self.processes[zmqport]['pool_worker'] is not None:
                done = False
                if self.processes[zmqport]['pool_worker'].ready():
                    fitnesses = self.processes[zmqport]['pool_worker'].get()
                    if fitnesses is None:
                        self.failed_genomes.append(self.processes[zmqport]['genomes'][0])
                    else:
                        for i, fitness in enumerate(fitnesses):
                            if type(self.processes[zmqport]['genomes'][i].fitness) is not list:
                                self.processes[zmqport]['genomes'][i].fitness = []
                            self.processes[zmqport]['genomes'][i].fitness.append(fitness)
                            # keep as list for now
                    self.processes[zmqport]['genomes'] = None
                    self.processes[zmqport]['pool_worker'] = None
                    if fitnesses is None:  # do this anyway
                        print()
                        print('failed genome:', self.processes[zmqport]['genome order'])
                    if debug and fitnesses is not None:
                        print()
                        print('got genome:', self.processes[zmqport]['genome order'])
        return done

    def inner_loop(self,
                   genomes,
                   config,
                   TRIALS,
                   evaluate_each_gen,
                   sleeptime,
                   reset_after_first_fail,
                   zmq_def_port,
                   websocket_def_port,
                   port_step,
                   headless,
                   debug):
        """
        looops through genomes, creates an experiment with that network, and sets the genome fitness

        @param genomes: genomes to evaluate
        @param config: config to use
        @param TRIALS: trials to evaluate each genome
        @param evaluate_each_gen: irrelevant for this class
        @param sleeptime: amount to sleep after important commands
        @param reset_after_first_fail: if we reset all coppeliasims after one failure to connect
        @param zmq_def_port: default port for ZMQ api
        @param websocket_def_port: default port for websocket
        @param port_step: amount to increment ports for different coppelia instances
        @param headless: whether to run coppelia in headless mode
        @param debug: whether to print excessive debug messages
        @return: number of tries it took to finish all genomes
            if more than 1, some error probably happened
        """

        pool = Pool(processes=self.current_num_sims)
        self.failed_genomes = []
        tries = 0
        global_polulation = []
        for genome_id, genome in genomes:
            global_polulation.append(genome)
            if not self.just_restored:
                genome.fitness = None
                # needs to be set manually because sometimes it carries over
        while tries == 0 or self.failed_genomes:
            self.activate_rclpy(sleeptime=sleeptime)
            self.make_coppelia_sims(zmq_def_port=zmq_def_port,
                                    websocket_def_port=websocket_def_port,
                                    port_step=port_step,
                                    headless=headless)
            tries += 1
            j = 0
            if tries == 1:
                todo = global_polulation
            else:
                todo = tuple(self.failed_genomes)
                self.failed_genomes = []
            for genome in todo:
                j += 1
                skip = False
                if self.just_restored:
                    # if we just restored, we can skip evaluating this generation
                    skip = True
                if (not skip) and reset_after_first_fail and self.failed_genomes:
                    self.failed_genomes.append(genome)
                    skip = True

                print(('skipping' if skip else 'evaluating') + ' genome ' + str(j) + '/' + str(len(todo)),
                      end=('\n' if debug else '\r'))
                if skip:
                    continue

                # create an ecosystem
                eco = [genome]
                while len(eco) < self.num_agents:
                    eco.append(global_polulation[np.random.randint(0, len(global_polulation))])

                # for each genome, assign a port, and create a process
                # the processes will finish after running the experiment
                port_assigned = None
                while port_assigned is None:
                    # loop to collect results
                    self.collect_genome_fitnesses(debug=debug)

                    # loop to start new self.processes
                    for zmqport in self.processes:
                        if self.processes[zmqport]['genomes'] is None:
                            self.processes[zmqport]['genomes'] = eco
                            self.processes[zmqport]['genome order'] = j
                            self.processes[zmqport]['pool_worker'] = pool.apply_async(ecosystem_eval_genoms,
                                                                                      args=
                                                                                      [
                                                                                          self.exp_maker,
                                                                                          eco,
                                                                                          config,
                                                                                          zmqport,
                                                                                          TRIALS,
                                                                                      ]
                                                                                      )
                            port_assigned = zmqport
                            break
                    time.sleep(sleeptime)
            # now make sure all processes are done
            while not self.collect_genome_fitnesses(debug=debug):
                # will run until all processes are done
                # i.e. when collect_genome_fitnesses returns True
                time.sleep(sleeptime)

            if self.failed_genomes:
                tries += 1
                print()
                print("FAILED SOME GENOME, TRYING AGAIN, time number " + str(tries))
            self.close_coppelia_sims()
            self.shutdown_rclpy(sleeptime=sleeptime)
        for genome_id, genome in genomes:
            genome.fitness = np.mean(genome.fitness)
        print()
        pool.close()
        return tries

    ####################################################################################################################
    # output functions
    ####################################################################################################################
    def result_of_experiment(self, trials=1, gen_indices=(-1,), network_to_use=None, display=True, zmqport=23000,
                             websocket_port=23050):
        """
        runs an experiment with best genome found, returns results

        @param trials: number of trials to run
        @param gen_indices: which generations to show, defaults to just last one
        @param network_to_use: if defined, use this network to act in environment
            function with the same inputs and outputs of the neural network in the environment
            leave as None if we want to use the evolved network
        @param display: whether to open coppelia GUI
        @param zmqport: port to use for zmq
        @param websocket_port: port to use for websocket
        @return: result of src.Experiment.experiments
        """
        if self.MOST_RECENT(self.checkpt_dir) is None:
            raise Exception("DIRECTORY EMPTY: " + self.checkpt_dir)

        cmd = COPPELIA_WAKEUP + ('' if display else ' -h') + \
              ' -GwsRemoteApi.port=' + str(websocket_port) + \
              ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
        wakeup = [cmd]
        all_goals = []
        for index in gen_indices:
            path = os.path.join(self.checkpt_dir, self.MOST_RECENT(self.checkpt_dir)[index])
            p = self.restore_checkpoint(path)
            print("DISPLAYING:", path)
            global_population = []
            for genome_id in p.population:
                global_population.append(p.population[genome_id])
            for t in range(trials):
                eco = []
                for i in range(self.num_agents):
                    eco.append(global_population[np.random.randint(0, len(global_population))])
                if network_to_use is None:
                    networks = [neat.nn.FeedForwardNetwork.create(genome, self.config) for genome in eco]
                    net_fun = lambda i: networks[i]
                else:
                    net_fun = lambda i: network_to_use

                exp: blimpNet = self.exp_maker(nets=net_fun, wakeup=wakeup, port=zmqport)
                goals = exp.experiments(trials=1)
                exp.kill()
                all_goals.append(goals)
        return all_goals


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
