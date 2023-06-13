from swarm_expiriment import *


class blimpNet(BlimpExperiment):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 sim=None,
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
        @param wakeup: code to run in command line before starting experiment
        @param sleeptime: time to wait before big commands (i.e. stop simulation, start simulation, pause simulation)
        """
        super().__init__(num_agents,
                         start_zone,
                         scenePath,
                         blimpPath,
                         sim=sim,
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
                 wakeup=None,
                 sleeptime=1.):
        super().__init__(num_agents,
                         start_zone,
                         scenePath,
                         blimpPath,
                         networkfn,
                         sim=sim,
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

class octantBlimp(blimpNet):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfn,
                 sim=None,
                 wakeup=None,
                 sleeptime=1.):
        super().__init__(num_agents,
                         start_zone,
                         scenePath,
                         blimpPath,
                         networkfn,
                         sim=sim,
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
from matplotlib import pyplot as plt

DIR = os.path.join(os.getcwd(), os.path.dirname(sys.argv[0]))
config_file = os.path.join(DIR, 'config', 'test-config-feedforward')
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_file)


def eval_genomes(genomes,
                 config):
    for genome_id, genome in genomes:
        genome.fitness = .0
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        exp = xyzBlimp(num_agents=1,
                       start_zone=lambda i: (0, 0, 1),
                       scenePath=empty_path,
                       blimpPath=narrow_blimp_path,
                       networkfn=net.activate,
                       sleeptime=.01)
        trials = 1
        goals = exp.experiments(trials=trials, end_time=lambda t: t > 1)
        genome.fitness += sum(goals) / trials


p = neat.Population(config)
p.add_reporter(neat.StdOutReporter(True))
stats = neat.StatisticsReporter()
p.add_reporter(stats)
winner = p.run(eval_genomes, 10)
print('\nBest genome:\n{!s}'.format(winner))

print('\nOutput:')
winner_net = neat.nn.FeedForwardNetwork.create(winner, config)

exp = xyzBlimp(num_agents=1,
               start_zone=lambda i: (0, 0, 1),
               scenePath=empty_path,
               blimpPath=narrow_blimp_path,
               networkfn=winner_net.activate)
input()
trials = 10
goals = exp.experiments(trials=trials, end_time=lambda t: t > 10)
print(goals)
