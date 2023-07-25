import os, sys
#from matplotlib import pyplot as plt

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

def ckpt_dir_from_name(name):
    return os.path.join(DIR, 'evolution', 'checkpoints', name)


def config_path_from_name(name):
    return os.path.join(DIR, 'evolution', 'config', name)



