from evolution.evolutionBlimp import EvolutionExperiment, eval_genom

class ComparisonExperiment(EvolutionExperiment):
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
            self.failure = {k: True for k in self.bandits}
        else:
            self.bandits = {num_simulators: []}
            self.failure = {num_simulators: True}
        self.current_num_sims = num_simulators
        if restore and self.MOST_RECENT(self.checkpt_dir) is not None:
            print('RESTORING')
            #TODO: restore better
            p = self.restore_checkpoint(os.path.join(self.checkpt_dir, self.MOST_RECENT(self.checkpt_dir)[-1]))
            self.just_restored = True
        else:
            # todo: initialize population
            p = better_Population(self.config)
        p.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        p.add_reporter(stats)
        p.add_reporter(
            neat.checkpoint.Checkpointer(checkpt_freq,
                                         filename_prefix=os.path.join(self.checkpt_dir, 'neat-checkpoint-')))
        # todo: while generations, run eval_genom to grab fitnesses
        # or repeatedly use self.result_of_expiriment with param network_to_use
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