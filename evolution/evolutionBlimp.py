from src.network_blimps import *
import os
import threading
import neat
from evolution.better_pop import better_Population
import gzip, random, pickle

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
config_file = os.path.join(DIR, 'config', 'test-config-feedforward')
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_file)


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


L = 3
K = 4
INPUT_DIM = L * K


def START_ZONE(i):
    return ((-2, 2), (-2, 2), (1, 3))


def exp_make(net, sim=None, port=23000, wakeup=None):
    return xy_zero_Blimp(num_agents=5,
                         start_zone=START_ZONE,
                         scenePath=empty_path,
                         blimpPath=narrow_blimp_path,
                         networkfn=net.activate,
                         height_range=(1, 1),
                         use_ultra=True,
                         sim=sim,
                         simId=port,
                         wakeup=wakeup
                         )
    return l_k_tant_clump_blimp(num_agents=5,
                                start_zone=START_ZONE,
                                scenePath=empty_path,
                                blimpPath=narrow_blimp_path,
                                networkfn=net.activate,
                                sim=sim,
                                simId=port,
                                sleeptime=.01,
                                l=L,
                                k=K,
                                wakeup=wakeup)


TRIALS = 2
END = lambda t: t > 10
NAME = 'xy_zero_test'
CHECKPT_DIR = os.path.join(DIR, 'checkpoints', NAME)


def MOST_RECENT(dir=CHECKPT_DIR):
    out = None
    if os.path.exists(dir):
        for fil in os.listdir(dir):
            if out is None:
                out = fil
            else:
                out = max(fil, out, key=lambda f: int(f[f.rindex('-') + 1:]))
    return out


if not os.path.exists(CHECKPT_DIR):
    os.makedirs(CHECKPT_DIR)


def restore_checkpoint(filename):
    """Resumes the simulation from a previous saved point."""
    with gzip.open(filename) as f:
        generation, config, population, species_set, rndstate = pickle.load(f)
        random.setstate(rndstate)
        return better_Population(config, (population, species_set, generation))


def eval_genom(genome, config, port, dict_to_unlock, key='locked', sim=None):
    if genome.fitness is not None:
        return
    genome.fitness = .0
    net = neat.nn.FeedForwardNetwork.create(genome, config)

    exp = exp_make(net, sim, port)
    goals = exp.experiments(trials=TRIALS, end_time=END)
    genome.fitness += sum(goals) / TRIALS
    dict_to_unlock[key] = False
    del exp
    # print(genome.fitness)


def eval_genomes(genomes,
                 config,
                 num_simulators=10,
                 open_coppelia=True,
                 # this will open a bunch of coppelia sims on startup, if this is false, it just assumes they are open
                 # in the correct ports
                 headless=True,
                 port_step=2,
                 coppelia='/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh',
                 zmq_def_port=23000,
                 websocket_def_port=23050,
                 close_after=True,
                 sleeptime=.1,
                 resttime=.1,
                 ):
    while True:
        try:
            if not rclpy.ok():
                rclpy.init()
            break
        except:
            time.sleep(sleeptime)
    processes = dict()
    for i in range(num_simulators):
        zmqport = zmq_def_port + port_step * i

        if open_coppelia:
            processes[zmqport] = dict()
            cmd = coppelia + (' -h' if headless else '') + \
                  ' -GwsRemoteApi.port=' + str(websocket_def_port + port_step * i) + \
                  ' -GzmqRemoteApi.rpcPort=' + str(zmqport)
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            processes[zmqport]['subproc'] = p
            processes[zmqport]['pid'] = p.pid
    for zmqport in processes:
        # must be done when simulator is running
        from zmqRemoteApi import RemoteAPIClient
        client = RemoteAPIClient(port=zmqport)
        processes[zmqport]['client'] = client
        processes[zmqport]['sim'] = client.getObject('sim')
        processes[zmqport]['locked'] = False  # if the simulator is being used for something
    start_time = time.time()
    for genome_id, genome in genomes:
        # for each genome, assign a port, and create a thread
        # the ports should unlock as soon as an earlier thread is done with them
        port_assigned = None
        while port_assigned is None:
            for zmqport in processes:
                if not processes[zmqport]['locked'] and ('thread' not in processes[zmqport] or
                                                         not processes[zmqport]['thread'].is_alive()):
                    processes[zmqport]['locked'] = True
                    th = threading.Thread(target=lambda: eval_genom(genome=genome,
                                                                    config=config,
                                                                    port=zmqport,
                                                                    dict_to_unlock=processes[zmqport],
                                                                    sim=processes[zmqport]['sim']
                                                                    ),
                                          # daemon=True
                                          )
                    th.start()
                    time.sleep(sleeptime)
                    port_assigned = zmqport
                    processes[zmqport]['thread'] = th
                    break
    # now make sure all threads are done
    done = False
    while not done:
        done = True
        for zmqport in processes:
            if 'thread' in processes[zmqport] and processes[zmqport]['thread'].is_alive():
                done = False
        time.sleep(sleeptime)
    # maybe close the coppelia processes
    dt = time.time() - start_time
    if close_after:
        for zmqport in processes:
            kill(processes[zmqport]['pid'])

        while True:
            try:
                if rclpy.ok():
                    # this fixes stuff??
                    # TODO: looks like the best way is to verify it works for one generation, then fully reset everything each gen
                    rclpy.shutdown()
                break
            except:
                time.sleep(sleeptime)
    time.sleep(resttime)


if True:
    if MOST_RECENT() is not None:
        p = restore_checkpoint(os.path.join(CHECKPT_DIR, MOST_RECENT()))
    else:
        p = better_Population(config)
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)
    p.add_reporter(neat.checkpoint.Checkpointer(1, filename_prefix=os.path.join(CHECKPT_DIR, 'neat-checkpoint-')))
    winner = p.run(eval_genomes, 2)
    print('\nBest genome:\n{!s}'.format(winner))
    print('\nOutput:')
    winner_net = neat.nn.FeedForwardNetwork.create(winner, config)
    if False:
        exp = exp_make(winner_net,
                       wakeup=['/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh'])
        input("ENTER TO START")
        trials = 2
        goals = exp.experiments(trials=trials, end_time=END)
        print(goals)
        exp.kill()
p = Checkpointer_hardly_knower.restore_checkpoint(os.path.join(CHECKPT_DIR, MOST_RECENT()))

quit()
winner_net = p.best_genome

exp = exp_make(winner_net,
               wakeup=[COPPELIA_WAKEUP])
goals = exp.experiments(trials=2, end_time=END)
print(goals)
exp.kill()
