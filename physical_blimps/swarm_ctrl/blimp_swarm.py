from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import read_config
from swarm_ctrl.vicon import Vicon
from CONFIG import *
import numpy as np


class viconBlimps(BlimpManager):
    def __init__(self,
                 cfg_paths,
                 vicon: Vicon,  # IMPORTANT: vicon's object with id 'i' must correspond to blimp cfg_paths[i]
                 port=ALIEN_PORT,
                 logger=False):
        """
        BlimpManager with added vicon interface so we can grab the positions of the blimps as well
            THIS USES GLOBAL STUFF to make everything less of a headache
            so sending a blimp towards (1,1) will always send it in the same direction regardless or rotation

        @param cfg_paths: list of config yaml file paths to interface the blimps
            we will call pyBlimp.utils.read_config on this list then pass it to BlimpManager
        @param vicon: a Vicon object
            @NOTE: vicon's object with id 'i' must correspond to blimp cfg_paths[i]
        @param port: USB port that that alien thingy is plugged into
            this should be specified in CONFIG.py, and not really necessary to change
        @param logger: probably what it sounds like
        """
        cfg = read_config(cfg_paths)
        self.num_blimps = len(cfg_paths)
        super().__init__(cfg=cfg, port=port, logger=logger)
        self.vicon = vicon

    def get_position(self, agent_id):
        """
        returns position of blimp
        @param agent_id: agent id
        @return: R^3, position of blimp
        """
        return self.vicon.get_object_pos(agent_id)

    def get_head(self, agent_id):
        """
        returns heading of blimp
        @param agent_id: agent id
        @return: R, heading of blimp
        """
        return self.vicon.get_object_head(agent_id)

    def move_agent(self, agent_id, vec):
        """
        moves an agent

        @param agent_id: agent id to publish to
        @param vec: vector to send to the method we decide to use
        """
        return self.force_stability_command(agent_id=agent_id, vec=vec)

    def force_stability_command(self, agent_id, vec):
        """
        moves an agent

        @param agent_id: agent id to publish to
        @param vec: R^3 velocity? vector
            OR R^4 velocity? and angular velocity? vector

        @note: if given an R^3 vector, we will let make turn command be to stabilize the blimp rotation
        """
        xy = vec[:2]
        theta = np.arctan2(vec[1], vec[0])%(2*np.pi)
        r = np.linalg.norm(xy)
        head = self.get_head(agent_id=agent_id)

        theta_p = theta - head

        cmd = np.zeros(4)

        # 'forward' command
        cmd[0] = r*np.cos(theta_p)
        # 'left' command
        cmd[1] = -r*np.sin(theta_p)
        # 'height' command
        cmd[2] = -vec[2]
        # 'rotation' command
        if len(vec) == 3:
            cmd[3] = self.get_stability_command(agent_id=agent_id)
        else:
            cmd[3] = -vec[3]
        self.set_cmd(cmd, agent_id)

    def get_stability_command(self, agent_id):
        """
        gets the command to stabilize the rotation of a certian blimp

        @param agent_id:
        @return: R, command to put in to try to make the blimp rotate as little as possible
        """
        return 0
