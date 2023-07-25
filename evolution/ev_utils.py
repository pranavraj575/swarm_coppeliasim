import os, sys
from evolution.evolutionBlimp import *

try:
    from matplotlib import pyplot as plt
except:
    print("WARNING: matplot lib import not working")

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))


def ckpt_dir_from_name(name):
    return os.path.join(DIR, 'evolution', 'checkpoints', name)


def config_path_from_name(name):
    return os.path.join(DIR, 'evolution', 'config', name)


def experiment_handler(args, save_name, config_name, exp_maker, Constructor):
    """
    handles running the experiment defined
        uses args to control what is done

    @param args: arg parser thingy
    @param save_name: name of checkpoint file, assumeed to be in swarm_coppeliasim/evolution/checkpoints
    @param config_name: name of config file, assumed to be in swarm_coppeliasim/evolution/config
    @param exp_maker: generates experiment,
        argument to be passed to Constructor
    @param Constructor: class to use, inherits GeneralEvolutionaryExperiment
    """
    config_file = config_path_from_name(config_name)
    checkpt_dir = ckpt_dir_from_name(save_name)
    print("SAVING TO:", checkpt_dir)

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

    if args.generations:
        port_step = args.port_step
        zmq_def_port = 23000 + port_step*args.offset
        websocket_def_port = 23050 + port_step*args.offset

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
        stuff = []
        std_key = None
        if args.plot_std:
            std_key = args.plot_std
        if args.plot_stat == 'all':
            if std_key is not None:
                raise Exception("cannot graph std as well as 'all' stats")
            sample = list(generation_dict.keys())[0]
            for key in generation_dict[sample]:
                if key != 'species':
                    stuff.append(key)
        else:
            stuff.append(args.plot_stat)
        for key in stuff:
            plot_name = key.replace(' ', '_') + ('' if std_key is None else '_std') + '.png'
            plot_key(generation_dict=generation_dict,
                     key_list=[key],
                     std_key_list=None if std_key is None else [std_key],
                     show=args.show,
                     file_path=os.path.join(PLOT_DIR, plot_name)
                     )
    if args.show:
        print(ee.result_of_experiment(gen_indices=(args.show_gen,)))


def plot_key(generation_dict, key_list, std_key_list=None, show=False, file_path=None):
    """
    plots a key from a dictionary, assuming the x values are the keys
    @param generation_dict: generation_dict[generation][key1][key2]... is the value we are plotting
    @param key_list: [key1,key2]... to access nested dictionaries for y value
    @param std_key_list: [key1,key2]... to access nested dictionaries for stdev value, None if ignore range
    @param show: whether to show plot
    @param file_path: path to save plot, None if no save
    @return: y values
    """
    try:
        key_list[0]
    except:
        key_list = [key_list]
    if std_key_list is not None:
        try:
            std_key_list[0]
        except:
            std_key_list = [std_key_list]

    X = list(generation_dict.keys())
    X.sort()
    Y = []
    STD = []
    for x in X:
        val = generation_dict[x]
        for key in key_list:
            val = val[key]
        Y.append(val)
        if std_key_list is not None:
            std = generation_dict[x]
            for key in std_key_list:
                std = std[key]
            STD.append(std)
    plt.plot(X, Y)
    if file_path is not None:
        plt.savefig(file_path)
    if show:
        plt.show()
    else:
        plt.close()
