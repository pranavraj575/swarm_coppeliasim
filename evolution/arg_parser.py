import argparse

PARSER = argparse.ArgumentParser()
PARSER.add_argument("-a", "--agents", type=int, required=False, default=20,
                    help="Specify number of agents")
PARSER.add_argument("-g", "--generations", type=int, required=False, default=0,
                    help="generations to train for")

PARSER.add_argument("--trials", type=int, required=False, default=1,
                    help="trials to evaluate each genome")

PARSER.add_argument("--create", action="store_true", required=False,
                    help="whether to create new directory")

PARSER.add_argument("--num_sims", type=int, required=False, default=8,
                    help="number of simulators to use for training")
PARSER.add_argument("--sims_low", type=int, required=False, default=-1,
                    help="low bound of num_sims to try")
PARSER.add_argument("--sims_high", type=int, required=False, default=-1,
                    help="high bound of num_sims to try")

PARSER.add_argument("--offset", type=int, required=False, default=0,
                    help="offset port number (should be number of simulators already in use)")
PARSER.add_argument("--port_step", type=int, required=False, default=2,
                    help="ports to skip for each new coppeliasim instance")
PARSER.add_argument("--overwrite", action="store_true", required=False,
                    help="whether to overwrite start instead of starting at recent checkpoint")

PARSER.add_argument("--show", action="store_true", required=False,
                    help="whether to show graphics, run this with -g 0 to show result")
PARSER.add_argument("--show_gen", type=int, required=False, default=-1,
                    help="if --show is on, choose which generation to display at end (defaults to last)")
PARSER.add_argument("--show_optimal", action="store_true", required=False,
                    help="whether to display hard coded optimal policy")
PARSER.add_argument("--show_stats", action="store_true", required=False,
                    help="whether to show statistics at end (run with -g 0 to just show stats)")
PARSER.add_argument("--show_random", action="store_true", required=False,
                    help="whether to show a random genome for --show and --collect_results")

PARSER.add_argument("--collect_results", action="store_true", required=False,
                    help="whether to save fitness results to a file")
PARSER.add_argument("--num_to_collect", type=int, required=False, default=60,
                    help="number of results to collect (defaults to 30)")


PARSER.add_argument("--plot_stat", action="store", required=False, default='',
                    help="saves a graph of a specific stat under the associated checkpoints folder, --graph_stat all to graph all")
PARSER.add_argument("--plot_std", action="store", required=False, default='',
                    help="to be used with --graph stat, graphs this key as standard deviation")

PARSER.add_argument("--debug", action="store_true", required=False,
                    help="whether to run in debug mode")


def check_basic(args):
    if args.sims_low >= 1:
        if not args.sims_low <= args.num_sims or not args.num_sims < args.sims_high:
            raise Exception("make sure sims_low <= num_sims < sims_high")
    if args.num_sims < 1:
        raise Exception("need a sim")
    if args.port_step < 1:
        raise Exception("port step should be at least 1")
    if args.port_step < 2:
        print("WARNING: probably make port step at least 2")
    return True
