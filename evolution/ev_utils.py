import os, sys
from evolution.evolutionBlimp import *

try:
    from matplotlib import pyplot as plt
except:
    print("WARNING: matplot lib import not working")

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

PLOT_KEYS = ['best_fitness', 'min_fitness']
PAIR_KEYS = [('mean_fitness', 'stdev_fitness')]
VALID_KEYS = list(tuple(PLOT_KEYS))
for mean, std in PAIR_KEYS:
    VALID_KEYS.append(mean)
    VALID_KEYS.append(std)


def ckpt_dir_from_name(name):
    return os.path.join(DIR, 'evolution', 'checkpoints', name)


def config_path_from_name(name):
    return os.path.join(DIR, 'evolution', 'config', name)


def experiment_handler(args, save_name, config_name, exp_maker, Constructor, optimal_policy):
    """
    handles running the experiment defined
        uses args to control what is done

    @param args: arg parser thingy
    @param save_name: name of checkpoint file, assumeed to be in swarm_coppeliasim/evolution/checkpoints
    @param config_name: name of config file, assumed to be in swarm_coppeliasim/evolution/config
    @param exp_maker: generates experiment,
        argument to be passed to Constructor
    @param Constructor: class to use, inherits GeneralEvolutionaryExperiment
    @param optimal_policy: takes in neural network inputs, outputs the command vector
        should be hard coded 'optimal policy' to use when --show is on
    """
    config_file = config_path_from_name(config_name)
    checkpt_dir = ckpt_dir_from_name(save_name)
    print("CHECKPOINT DIR:", checkpt_dir)

    if not os.path.exists(checkpt_dir):
        if args.create:
            os.makedirs(checkpt_dir)
        else:
            raise Exception("DIRECTORY DOES NOT EXIST (try running with --create): " + checkpt_dir)

    if Constructor is EvolutionExperiment:
        ee = Constructor(checkpt_dir=checkpt_dir,
                         exp_maker=exp_maker,
                         config_file=config_file)
    elif Constructor is EcosystemEvolutionExperiment:
        ee = Constructor(checkpt_dir=checkpt_dir,
                         ecosystem_exp_maker=exp_maker,
                         num_agents=args.agents,
                         config_file=config_file)
    else:
        raise Exception("constructor must be one of the above")

    port_step = args.port_step
    zmq_def_port = 23000 + port_step*args.offset
    websocket_def_port = 23050 + port_step*args.offset

    if args.generations:
        ee.train(generations=args.generations,
                 TRIALS=args.trials,
                 num_simulators=args.num_sims,
                 headless=not args.show,
                 restore=not args.overwrite,
                 evaluate_each_gen=True,
                 zmq_def_port=zmq_def_port,
                 websocket_def_port=websocket_def_port,
                 port_step=port_step,
                 num_sim_range=None if args.sims_low < 1 else (args.sims_low, args.sims_high),
                 debug=args.debug
                 )
    if args.show_stats:
        ee.show_stats()
    if args.plot_stat:
        generation_dict = ee.get_stats()
        PLOT_DIR = os.path.join(checkpt_dir, 'plots')
        if not os.path.exists(PLOT_DIR):
            os.makedirs(PLOT_DIR)
        key_list = []
        std_key_list = []
        plot_name = ''
        if args.plot_stat == 'all':
            key_list = PLOT_KEYS + [mean for mean, _ in PAIR_KEYS],
            std_key_list = [None for _ in PLOT_KEYS] + [std for _, std in PAIR_KEYS]
            plot_name = 'all.png'
        else:
            if args.plot_stat not in VALID_KEYS:
                raise Exception("--plot_stat arg not valid, possible options are: " + str(VALID_KEYS))

            if args.plot_std:
                std_key = args.plot_std
            else:
                std_key = None
            if std_key is not None:
                if args.plot_std not in VALID_KEYS:
                    raise Exception("--plot_std arg not valid, possible options are: " + str(VALID_KEYS))
            key_list.append(args.plot_stat)
            std_key_list.append(std_key)
            plot_name = args.plot_stat.replace(' ', '_') + ('' if std_key is None else '_std') + '.png'

        plot_key(generation_dict=generation_dict,
                 key_list=key_list,
                 std_key_list=std_key_list,
                 show=args.show,
                 file_path=os.path.join(PLOT_DIR, plot_name)
                 )
    if args.show:
        if args.show_optimal:
            alt_network = optimal_policy
        else:
            alt_network = None
        print(ee.result_of_experiment(gen_indices=(args.show_gen,),
                                      network_to_use=alt_network,
                                      zmqport=zmq_def_port,
                                      websocket_port=websocket_def_port))


def plot_key(generation_dict, key_list, std_key_list=None, show=False, file_path=None, title=None):
    """
    plots a key from a dictionary, assuming the x values are the keys
    @param generation_dict: generation_dict[generation][key1][key2]... is the value we are plotting
    @param key_list: [key1,key2]... dictionary keys to plot
    @param std_key_list: [key1,key2]... to access stdev value, None if ignore range
    @param show: whether to show plot
    @param file_path: path to save plot, None if no save
    @param title: title of graph, None if None
    @return: y values
    """

    if std_key_list is None:
        std_key_list = [None for _ in range(len(key_list))]
    X = list(generation_dict.keys())
    X.sort()
    legend = []

    for i, key in enumerate(key_list):
        value_name = key.replace('_', ' ').capitalize()
        legend.append(value_name)
        Y = []
        STD = []
        for x in X:
            val = generation_dict[x][key]
            Y.append(val)
            std_key = std_key_list[i]
            if std_key is not None:
                std = generation_dict[x][std_key]
                STD.append(std)
        Y = np.array(Y)
        plt.plot(X, Y)
        if STD:
            plt.fill_between(X, Y - STD, Y + STD, alpha=.3)
            legend.append("$\\pm1$ stdev")
    plt.xlabel('Generation')
    plt.ylabel('Fitness')
    if title:
        plt.title(title)
    plt.legend(legend)
    if file_path is not None:
        plt.savefig(file_path)
    if show:
        plt.show()
    else:
        plt.close()


def auto_plotter_hardly_know_her(directory):
    """
    plots graphs for every folder in the directory

    @param directory: structure is [directory/<exp name>/neat-checkpoint-69] 
    """
    for folder in os.listdir(directory):
        fake = GeneralEvolutionaryExperiment(checkpt_dir=os.path.join(directory, folder), exp_maker=None,
                                             config_file=os.path.join(DIR, 'evolution', 'config',
                                                                      'test-config-feedforward'))
        try:
            print('plotting:', folder)
            gen_dict = fake.get_stats()
            PLOT_DIR = os.path.join(directory, folder, 'plots')
            if not os.path.exists(PLOT_DIR):
                os.makedirs(PLOT_DIR)
            sample = list(gen_dict.keys())[0]
            keys = list(gen_dict[sample].keys())
            for key in keys:
                if key != 'species':
                    plot_name = key.replace(' ', '_') + '.png'
                    plot_key(generation_dict=gen_dict,
                             key_list=[key],
                             std_key_list=None,
                             show=False,
                             file_path=os.path.join(PLOT_DIR, plot_name))
            plot_key(generation_dict=gen_dict,
                     key_list=['mean_fitness'],
                     std_key_list=['stdev_fitness'],
                     show=False,
                     file_path=os.path.join(PLOT_DIR, 'mean_fitness_std.png'))
            plot_key(generation_dict=gen_dict,
                     key_list=PLOT_KEYS + [mean for mean, _ in PAIR_KEYS],
                     std_key_list=[None for _ in PLOT_KEYS] + [std for _, std in PAIR_KEYS],
                     show=False,
                     file_path=os.path.join(PLOT_DIR, 'all.png'))
        except:
            print('failed:', folder)
            continue


if __name__ == "__main__":
    auto_plotter_hardly_know_her(os.path.join(DIR, 'evolution', 'checkpoints'))
