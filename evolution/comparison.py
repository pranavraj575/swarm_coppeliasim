from evolution.evolutionBlimp import EvolutionExperiment
import numpy as np
import os,copy

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

    exp = exp_maker(net=genome, sim=None, port=port, wakeup=None)
    goals = exp.experiments(trials=TRIALS)
    if goals is None:
        # process failed
        fitness = None
    else:
        fitness = sum(goals)/TRIALS
    exp.close_zmq()
    del exp
    return fitness

class FSM:
    def __init__(self, states, transitions, num_conditions, init_state=None):
        """
        Args:
            states: list of states
            transitions: dict((state, condition idx)->probability dist over states)
            num_conditions: number of possible conditions
            init_state: initial state (if none, random)
        """
        self.states = states
        self.trans = transitions
        self.num_conditions = num_conditions
        if init_state is None:
            self.state = list(self.states)[np.random.randint(len(self.states))]
        else:
            self.state = init_state

    def step(self, condition):
        next_state = np.where(np.random.multinomial(1, self.trans[self.state, condition]))[0].item()
        self.state = next_state

    def clone(self):
        return FSM(states=self.states,
                   transitions=self.trans,
                   num_conditions=self.num_conditions,
                   init_state=copy.deepcopy(self.state),
                   )

    def produce_asexual(self, epsilon=.1):
        """
        reproduces self, distribution shaken by epsilon
        Args:
            epsilon:

        Returns:

        """
        # reproduce with noise
        trans = {key: self.trans[key] + np.random.normal(np.zeros(len(self.trans[key])), epsilon)
                 for key in self.trans}
        # normalize
        trans = {key: trans[key]/np.sum(trans[key]) for key in trans}

        return FSM(states=self.states,
                   transitions=trans,
                   num_conditions=self.num_conditions,
                   init_state=self.state,
                   )

    @staticmethod
    def breed(fsms):
        """
        breeds fsms that all have same states and num_conditions
        Args:
            fsms: list of fsms
        Returns:
            fsm with transitions randomly chosn among each
        """

        def rand_fsm():
            return fsms[np.random.randint(len(fsms))]

        states = rand_fsm().states
        init_state = rand_fsm().state
        trans = {key: rand_fsm().trans[key].copy() for key in rand_fsm().trans}
        num_conditions = rand_fsm().num_conditions
        return FSM(
            states=states,
            transitions=trans,
            num_conditions=num_conditions,
            init_state=init_state,
        )

    @staticmethod
    def random_fsm(states, num_conditions):
        """
        Args:
            states: list of states
            num_conditions: number of possible conditions
        Returns:
            FSM with random params
        """

        trans = dict()
        for state in states:
            for condition in range(num_conditions):
                trans_prob = np.random.rand(len(states))
                trans[state, condition] = trans_prob/np.sum(trans_prob)
        return FSM(states=states, transitions=trans, num_conditions=num_conditions)


class automode_soln:
    def __init__(self, constituents, map_to_condition_idx, num_conditions, fsm=None):
        """
        Args:
            constituents: constituent behaviors
            map_to_condition_idx: (obs->cond idx)
            num_conditions: mumber of possible condiditons
            fsm: fsm, if none makes random
                states must be indices of constituents
        """
        if fsm is None:
            self.fsm = FSM.random_fsm(states=list(range(len(constituents))), num_conditions=num_conditions)
        else:
            self.fsm = fsm
        self.map_to_condidtion = map_to_condition_idx
        self.constituents = constituents

    def policy(self, inputs):
        condition_idx = self.map_to_condidtion(inputs)
        self.fsm.step(condition_idx)
        behavior = self.constituents[self.fsm.state]
        return behavior(inputs)


class ComparisonExperiment(EvolutionExperiment):
    def __init__(self, checkpt_dir, exp_maker, config_file):
        """

        experiment to use NEAT evolutionary algorithm on a Experiment class (specifically a blimpNet)

        @param checkpt_dir: folder name of experiment, used for checkpoint directories and maybe for config file
        @param exp_maker: (net,sim,port,wakeup) -> blimpFSM
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
            # TODO: restore better
            p = self.restore_checkpoint(os.path.join(self.checkpt_dir, self.MOST_RECENT(self.checkpt_dir)[-1]))
            self.just_restored = True
        else:
            # todo: initialize population
            population = self.config.pop_size

        # todo: while generations, run eval_genom to grab fitnesses
        self.exp_maker()
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


if __name__ == '__main__':
    fsm = FSM.random_fsm(states=list(range(10)), num_conditions=10)
    print(fsm.state)
    print(fsm.trans[fsm.state, 2])

    fsm.step(2)
    print(fsm.state)
