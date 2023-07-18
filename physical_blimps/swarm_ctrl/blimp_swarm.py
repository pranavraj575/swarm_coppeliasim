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

    def position_command(self, agent_id, vec, update=True):
        """
        moves an agent to an input position vector

        @param agent_id: agent id to publish to
        @param vec: R^3 position vector
            OR R^4 position and angular heading
        @param update: whether to update the velocity of the agent before checking

        @note: if given an R^3 vector, we will let the turn command be to stabilize the blimp rotation
        """

        pos = self.vicon.get_object_pos(obj_id=agent_id, update=update)
        theta = self.vicon.get_object_head(obj_id=agent_id, update=False)
        vel = vec[:3] - pos
        if len(vec) == 3:
            return self.velocity_command(agent_id=agent_id, vec=vel, update=False)
        else:
            omega = vec[4] - theta
            return self.velocity_command(agent_id=agent_id, vec=np.concatenate((vel, [omega])), update=False)

    def velocity_command(self, agent_id, vec, update=True):
        """
        moves an agent to an input velocity vector

        @param agent_id: agent id to publish to
        @param vec: R^3 velocity vector
            OR R^4 velocity and angular velocity vector
        @param update: whether to update the velocity of the agent before checking

        @note: if given an R^3 vector, we will let the turn command be to stabilize the blimp rotation
        """
        vel = self.vicon.get_object_vel(obj_id=agent_id, update=update)
        omega = self.vicon.get_head_speed(obj_id=agent_id, update=False)
        acc = vec[:3] - vel
        if len(vec) == 3:
            return self.force_stability_command(agent_id=agent_id, vec=acc)
        else:
            ang_acc = vec[4] - omega
            return self.force_stability_command(agent_id=agent_id, vec=np.concatenate((acc, [ang_acc])))

    def force_stability_command(self, agent_id, vec):
        """
        moves an agent with an input force vector

        @param agent_id: agent id to publish to
        @param vec: R^3 force vector
            OR R^4 force and angular force vector

        @note: if given an R^3 vector, we will let the turn command be to stabilize the blimp rotation
        """
        EFFECT = [1., 1., 1., 1.]  # effective force for x,y,z,rotation given a command
        # found experimentally
        xy = vec[:2]
        theta = np.arctan2(vec[1], vec[0])%(2*np.pi)
        r = np.linalg.norm(xy)
        head = self.get_head(agent_id=agent_id)

        theta_p = theta - head

        cmd = np.zeros(4)

        # 'forward' command
        cmd[0] = r*np.cos(theta_p)/EFFECT[0]
        # 'left' command
        cmd[1] = -r*np.sin(theta_p)/EFFECT[1]
        # 'height' command
        cmd[2] = -vec[2]/EFFECT[2]
        # 'rotation' command
        if len(vec) == 3:
            cmd[3] = self.get_stability_command(agent_id=agent_id)/EFFECT[3]
        else:
            cmd[3] = -vec[3]/EFFECT[3]
        self.set_cmd(cmd, agent_id)

    def get_stability_command(self, agent_id, update=True):
        """
        gets the command to stabilize the rotation of a certian blimp

        @param agent_id:
        @param update: whether to update the information
        @return: R, command to put in to try to make the blimp rotate as little as possible
        """
        vel = self.vicon.get_head_speed(obj_id=agent_id, update=update)
        return -vel
