
import neat, os, sys
from matplotlib import pyplot as plt
import numpy as np

DIR = os.path.join(os.getcwd(), os.path.dirname(sys.argv[0]))
rng=6.28
cent=0
samples=50
grid=[(i/samples)*rng*2-rng+cent for i in range(samples+1)]

function=lambda x:[float(bool(x[0])^bool(x[1]) or x[2])]
get_inputs=lambda :[(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0)]
def eval_genomes(genomes,
                 config,
                 function=function,
                 get_inputs=get_inputs,
                 loss=lambda real,pred:-(real-pred)**2):
    for genome_id, genome in genomes:
        genome.fitness = .0
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        for xi in get_inputs():
            xo=function(xi)
            output = net.activate(xi)
            genome.fitness +=loss(output[0],xo[0])

config_file=os.path.join(DIR,'config','test-config-feedforward')
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_file)

# Create the population, which is the top-level object for a NEAT run.
p = neat.Population(config)

# Add a stdout reporter to show progress in the terminal.
p.add_reporter(neat.StdOutReporter(True))
stats = neat.StatisticsReporter()
p.add_reporter(stats)
#p.add_reporter(neat.Checkpointer(5))

# Run for up to 300 generations.
winner = p.run(eval_genomes, 600)

# Display the winning genome.
print('\nBest genome:\n{!s}'.format(winner))

# Show output of the most fit genome against training data.
print('\nOutput:')
winner_net = neat.nn.FeedForwardNetwork.create(winner, config)

for xi in get_inputs():

    output = winner_net.activate(xi)
    xo=function(xi)
    print("input {!r}, expected output {!r}, got {!r}".format(xi, xo, output))
quit()
nn=200
grid=[(i/nn)*rng*2-rng+cent for i in range(nn+1)]
plt.plot(grid,[winner_net.activate((xi,))[0] for xi in grid])
plt.plot(grid,[function(xi) for xi in grid])
plt.show()