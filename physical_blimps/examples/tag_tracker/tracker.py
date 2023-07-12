import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.at_utils import AprilTagHelper

if __name__ == "__main__":
    # setup exit on ctrl-c
    running = True
    def exit_handle(signum, frame):
        global running
        running = False
        
    signal.signal(signal.SIGINT, exit_handle)

    # load desired configs
    cfg_paths = ["configs/config2.yaml"]
    cfg = read_config(cfg_paths)

    # build the blimp object
    b = BlimpManager(cfg, "COM3", logger=False)

    # setup apriltag detector
    at = AprilTagHelper("utils/K_360x240.txt", "utils/dist_360x240.txt")

    # show the FPV
    fig, axes = plt.subplots(1,1)
    I = b.get_image(0)
    h = axes.imshow(I)
    axes.set_xticks([])
    axes.set_yticks([])
    axes.set_title("FPV")

    while running and b.get_running(0):
        # find the relative position of the apriltag
        I = b.get_image(0)
        tags, I_labeled = at.detect(I, label=True)

        # float in place if no tag detected
        des = np.zeros(4)
        des[3] = 1.5

        # decide inputs to track a single tag
        if len(tags) > 0:
            tag_pos = tags[0][2]
            distance = np.linalg.norm(tag_pos[:2])
            angle = np.arctan2(tag_pos[1], tag_pos[0])
            
            des[1] = -0.1*(0.5 - distance)
            des[2] = 0.5*angle + b.get_euler(0)[2]

        b.set_des(des, 0)

        # show the feed
        h.set_data(I_labeled)
        plt.draw(); plt.pause(0.0001)

        # break if the figure is closed
        if not plt.fignum_exists(fig.number): running = False

    b.shutdown()

