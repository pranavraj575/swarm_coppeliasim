import os
import numpy as np
import yaml

from numpy import pi
from scipy.spatial.transform import Rotation as R


def wrap(ang):
    return ang + 2*pi*(ang < -pi) - 2*pi*(ang > pi)


def quat2euler(quat):
    return R.from_quat(quat).as_euler('xyz')

    
def read_config(paths):
    cfg = []
    for p in paths:
        cfg.append(yaml.safe_load(open(p)))
        
    return cfg
