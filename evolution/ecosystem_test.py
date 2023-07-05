from src.network_blimps import *
from evolution.evolutionBlimp import EcosystemEvolutionExperiment
import argparse

class ecosystem_xy_wall_climb(xy_wall_climb_blimp):
    def __init__(self,
                 num_agents,
                 start_zone,
                 scenePath,
                 blimpPath,
                 networkfns,
                 height_range,
                 use_ultra,
                 end_time,
                 rng=2,
                 height_factor=.2,
                 sim=None,
                 simId=23000,
                 msg_queue=10,
                 wakeup=None,
                 sleeptime=.01,
                 spawn_tries=100):
        """
        ecosystem implementation of blimp wall climb
            this way is much easier than redefining everything as a ecosystemBlimpNet class
            the methods that need changing are goal_data and step

        """
        super().__init__(num_agents=num_agents,
                         start_zone=start_zone,
                         scenePath=scenePath,
                         blimpPath=blimpPath,
                         networkfn=networkfns,
                         height_range=height_range,
                         use_ultra=use_ultra,
                         end_time=end_time,
                         rng=rng,
                         height_factor=height_factor,
                         sim=sim,
                         simId=simId,
                         msg_queue=msg_queue,
                         wakeup=wakeup,
                         sleeptime=sleeptime,
                         spawn_tries=spawn_tries
                         )
        self.networks = self.network

    ####################################################################################################################
    # Expiriment functions
    ####################################################################################################################
    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        self.spin()
        for agent_id in self.agentData:
            network = self.networks(agent_id)
            z = network(self.get_network_input(agent_id))
            vec = self.get_vec_from_net_ouput(z, agent_id)
            self.move_agent(agent_id, vec)
        t = self.sim.getSimulationTime()
        # print('cycle time:',t-self.last_time,end='\r')
        self.last_time = t
        return self.end_test()

    def goal_data(self):
        val = super().goal_data()
        return [val for _ in range(self.num_agents)]

parser = argparse.ArgumentParser(description="for creating and running wall climbing evolutionary experiments")
parser.add_argument("-a", "--agents", type=int, required=False, default=20,
                    help="Specify number of agents")
parser.add_argument("-g", "--generations", type=int, required=False, default=0,
                    help="generations to train for")
parser.add_argument("--create", action="store_true", required=False,
                    help="whether to create new directory")

parser.add_argument("--height_lower", type=float, required=False, default=.8,
                    help="lower bound of height to hold blimps at")
parser.add_argument("--height_upper", type=float, required=False, default=1.2,
                    help="upper bound of height to hold blimps at")

parser.add_argument("--range", type=float, required=False, default=5.,
                    help="range to detect neighbors")

parser.add_argument("--xmin", type=float, required=False, default=.5,
                    help="x spawning lower bound (should be > 0 so they do not spawn on other side of wall)")
parser.add_argument("--xmax", type=float, required=False, default=8.,
                    help="x spawning upper bound")

parser.add_argument("--ymin", type=float, required=False, default=-4.,
                    help="y spawning upper bound")
parser.add_argument("--ymax", type=float, required=False, default=4.,
                    help="y spawning upper bound")

parser.add_argument("--zmin", type=float, required=False, default=.8,
                    help="z spawning upper bound")
parser.add_argument("--zmax", type=float, required=False, default=1.2,
                    help="z spawning upper bound")

parser.add_argument("--num_sims", type=int, required=False, default=8,
                    help="number of simulators to use for training")
parser.add_argument("--sims_low", type=int, required=False, default=-1,
                    help="low bound of num_sims to try")
parser.add_argument("--sims_high", type=int, required=False, default=-1,
                    help="high bound of num_sims to try")
parser.add_argument("--offset", type=int, required=False, default=0,
                    help="offset port number (should be number of simulators already in use)")
parser.add_argument("--port_step", type=int, required=False, default=2,
                    help="ports to skip for each new coppeliasim instance")
parser.add_argument("--overwrite", action="store_true", required=False,
                    help="whether to overwrite start instead of starting at recent checkpoint")
parser.add_argument("--show", action="store_true", required=False,
                    help="whether to show stats of all gens")
parser.add_argument("--show_result", action="store_true", required=False,
                    help="whether to show result at end")
args = parser.parse_args()
AGENTS = args.agents
gens = args.generations
RANGE = args.range
if args.sims_low >= 1:
    if not args.sims_low <= args.num_sims or not args.num_sims < args.sims_high:
        raise Exception("bruh")
END = 15
h_low = args.height_lower
h_upp = args.height_upper


def SPAWN_ZONE(i):
    return (args.xmin, args.xmax), (args.ymin, args.ymax), (args.zmin, args.zmax)


def eco_expe_make(nets, sim=None, port=23000, wakeup=None):
    return ecosystem_xy_wall_climb(num_agents=AGENTS,
                                          start_zone=SPAWN_ZONE,
                                          scenePath=caged_wall_climb_path,
                                          blimpPath=narrow_blimp_path,
                                          networkfns=lambda i:nets[i].activate,
                                          end_time=END,
                                          rng=RANGE,
                                          height_range=(h_low, h_upp),
                                          use_ultra=True,
                                          height_factor=1.,
                                          sim=sim,
                                          simId=port,
                                          wakeup=wakeup,
                                          sleeptime=.01)


save_name = 'ECOSYSTEM' + str(AGENTS) + \
            '_blimp_height_' + str(h_low).replace('.', '_') + "_to_" + str(h_upp).replace('.', '_') + \
            '_wall_climb_neighbor_rng_' + str(RANGE).replace('.', '_')
checkpt_dir = os.path.join(DIR, 'checkpoints', save_name)
print("SAVING TO:", checkpt_dir)

if not os.path.exists(checkpt_dir):
    if args.create:
        os.makedirs(checkpt_dir)
    else:
        raise Exception("DIRECTORY DOES NOT EXIST (try running with --create): " + checkpt_dir)
ee = EcosystemEvolutionExperiment(checkpt_dir=checkpt_dir,
                                  ecosystem_exp_maker=eco_expe_make,
                                  num_agents=AGENTS,
                                  config_name='blimp_wall')
if gens:
    port_step = args.port_step
    zmq_def_port = 23000 + port_step*args.offset
    websocket_def_port = 23050 + port_step*args.offset

    ee.train(generations=gens,
             TRIALS=1,
             num_simulators=args.num_sims,
             headless=not args.show,
             restore=not args.overwrite,
             evaluate_each_gen=True,
             zmq_def_port=zmq_def_port,
             websocket_def_port=websocket_def_port,
             port_step=port_step,
             num_sim_range=None if args.sims_low < 1 else (args.sims_low, args.sims_high)
             )
if args.show:
    ee.show_stats()
if args.show_result:
    print(ee.result_of_experiment())
