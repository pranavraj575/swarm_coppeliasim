import matplotlib.pyplot as plt
import numpy as np
import signal
import os, sys
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *

from CONFIG import *

BLIMP_DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CONFIGS_DIR = os.path.join(BLIMP_DIR, 'configs')

if __name__ == "__main__":
    # setup exit on ctrl-c
    running = True
    ids = [2]
    cfg_paths = []
    for id in ids:
        # create config file
        path = os.path.join(CONFIGS_DIR, 'config' + str(id) + '.yaml')
        f = open(path, 'w')
        f.write('id_num: ' + str(id) + '\n')
        f.write('my_ip: "' + MY_IP + '"\n')
        f.write('pi_ip: "' + IP_BLIMP_SUBNET + str(id + IP_BLIMP_INIT) + '"\n')
        for line in (
                'im_rows: 240',
                'im_cols: 360',
                'fps: 15',
                'qual: 15',
                '# motor board parameters',
                'positive_only: False',
                'k_r:  [0.45, 0., -1.7]',
                'k_p:  [0.45, 0., -1.7]',
                'k_yw: [0.002, 0.001, 0.07]',
                'k_pz: [0.35, 0.001, 0.055]',
        ):
            f.write(line)
            f.write('\n')
        f.close()
        cfg_paths.append(path)


    def exit_handle(signum, frame):
        global running
        running = False


    signal.signal(signal.SIGINT, exit_handle)

    # load desired configs
    cfg = read_config(cfg_paths)

    # build the blimp object
    b = BlimpManager(cfg, ALIEN_PORT, logger=False)

    # setup the joystick reader
    # js = JoyStick_helper()

    # show the FPV
    fig, axes = plt.subplots(1, 1)
    I = b.get_image(0)
    h = axes.imshow(I)
    axes.set_xticks([])
    axes.set_yticks([])
    axes.set_title("FPV")

    # manual/assisted mode flag
    manual_mode = True
    des = np.zeros(4)
    des[3] = 0
    t = time.time()
    while running and b.get_running(0) and time.time() - t <= 100:
        ax = [0., 0., .5, .2]
        if manual_mode:
            cmd = np.zeros(4)

            # set inputs
            cmd[0] = ax[1]
            cmd[1] = ax[0]
            cmd[2] = -ax[3]
            cmd[3] = ax[2]

            b.set_cmd(cmd, 0)
            cmd[3] = -cmd[3]
            # b.set_cmd(cmd, 1)

        else:
            # decide inputs
            des[0] = 0.5*ax[0]
            des[1] = 0.5*ax[1]
            des[2] = wrap(des[2] - 0.05*ax[2])
            des[3] = np.clip(des[3] + 0.05*ax[3], 0.0, 2.5)
            b.set_des(des, 0)

        # show the feed
        # print('state',b.get_state(0))
        I = b.get_image(0)
        h.set_data(I)
        plt.draw();
        plt.pause(0.0001)

        # break if the figure is closed
        if not plt.fignum_exists(fig.number): running = False

    b.shutdown()
