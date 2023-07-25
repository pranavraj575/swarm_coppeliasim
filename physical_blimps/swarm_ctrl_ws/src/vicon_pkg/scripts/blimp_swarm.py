#!/usr/bin/env python3
from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import read_config
from physical_blimps.swarm_ctrl_ws.src.vicon_pkg.scripts.vicon import Vicon
from physical_blimps.CONFIG import *
import numpy as np
import time


class viconBlimps(BlimpManager):
    def __init__(self,
                 cfg_paths,
                 vicon: Vicon,  # IMPORTANT: vicon's object with id 'i' must correspond to blimp cfg_paths[i]
                 command_type='force',
                 port=ALIEN_PORT,
                 logger=False,
                 ):
        """
        BlimpManager with added vicon interface so we can grab the positions of the blimps as well
            THIS USES GLOBAL STUFF to make everything less of a headache
            so sending a blimp towards (1,1) will always send it in the same direction regardless or rotation

        @param cfg_paths: list of config yaml file paths to interface the blimps
            we will call pyBlimp.utils.read_config on this list then pass it to BlimpManager
        @param vicon: a Vicon object
            @NOTE: vicon's object with id 'i' must correspond to blimp cfg_paths[i]
        @param command_type: from ['position', 'velocitiy', 'force'], determines what the move_agent function does
        @param port: USB port that that alien thingy is plugged into
            this should be specified in CONFIG.py, and not really necessary to change
        @param logger: hardly know her
        """
        cfg = read_config(cfg_paths)
        self.num_blimps = len(cfg_paths)
        super().__init__(cfg=cfg, port=port, logger=logger)
        self.vicon = vicon
        self.change_command_type(command_type)
        self.reset_agent_goals()

    ####################################################################################################################
    # init/shutdown functions
    ####################################################################################################################
    def reset_agent_goals(self):
        """
        resets agent goals if using position or velocity controller
        """
        self.agent_goals = [None for _ in range(self.num_blimps)]

    def change_command_type(self, new_type):
        """
        changes command type to the desired

        @param new_type: from ['position', 'velocitiy', 'force'], determines what the move_agent function does
        """
        if new_type not in {'position', 'velocity', 'force'}:
            raise Exception("invalid command type: " + str(new_type))
        self.command_type = new_type
        self.reset_agent_goals()

    ####################################################################################################################
    # state access functions
    ####################################################################################################################
    def get_position(self, agent_id):
        """
        returns position of blimp
        @param agent_id: agent id
        @return: R^3, position of blimp
        """
        return self.vicon.get_object_pos(agent_id)
    def get_velocity(self, agent_id):

        """
        returns velocty of blimp
        @param agent_id: agent id
        @return: R^3, velocity of blimp
        """
        return self.vicon.get_object_vel(agent_id)
    def get_acc(self, agent_id):

        """
        returns acceleration of blimp
        @param agent_id: agent id
        @return: R^3, acceleration of blimp
        """
        return self.vicon.get_object_acc(agent_id)

    def get_head(self, agent_id):
        """
        returns heading of blimp
        @param agent_id: agent id
        @return: R, heading of blimp
        """
        return self.vicon.get_object_head(agent_id)

    ####################################################################################################################
    # command functions
    ####################################################################################################################
    def movement_updates(self):
        """
        updates the movements of agents if using velocity or position controllers
            if the agents have not been asked to move, just make them still
        """
        for agent_id in range(self.num_blimps):
            if self.agent_goals[agent_id] is None:
                self.velocity_command(agent_id=agent_id, vec=np.zeros(3))
            else:
                self.move_agent(agent_id=agent_id, vec=self.agent_goals[agent_id])

    def move_agent(self, agent_id, vec):
        """
        moves an agent

        @param agent_id: agent id to publish to
        @param vec: vector to send to the method we decide to use
        """
        if self.command_type == 'position':
            return self.position_command(agent_id=agent_id, vec=vec)
        elif self.command_type == 'velocity':
            return self.velocity_command(agent_id=agent_id, vec=vec)
        elif self.command_type == 'force':
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
        self.agent_goals[agent_id] = vec
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
        self.agent_goals[agent_id] = vec
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
        self.agent_goals[agent_id] = vec  # why not
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
            cmd[3] = -self.get_stability_command(agent_id=agent_id)/EFFECT[3]
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
        return -vel*.001


class PhysicalExperiment(viconBlimps):
    def __init__(self,
                 cfg_paths,
                 vicon: Vicon,
                 command_type='velocity',
                 port=ALIEN_PORT,
                 logger=False):
        """
        Experiment skeleton using physical blimps

        @param cfg_paths: list of config yaml file paths to interface the blimps
            we will call pyBlimp.utils.read_config on this list then pass it to BlimpManager
        @param vicon: a Vicon object
            @NOTE: vicon's object with id 'i' must correspond to blimp cfg_paths[i]
        @param command_type: from ['position', 'velocitiy', 'force'], determines what the move_agent function does
        @param port: USB port that that alien thingy is plugged into
            this should be specified in CONFIG.py, and not really necessary to change
        @param logger: hardly know her
        """
        super().__init__(cfg_paths=cfg_paths,
                         vicon=vicon,
                         command_type=command_type,
                         port=port,
                         logger=logger)

    ####################################################################################################################
    # Expiriment functions (some need implementation in subclass)
    ####################################################################################################################
    def init_exp(self):
        """
        to be run at start of experiment, usually just resetting blimp positions
        """
        raise NotImplementedError()

    def step(self):
        """
        step to take continuously during an experiment
        (should probably include a pause, since this will be running continuously)

        @return: boolean, whether or not experiment is done
        """
        raise NotImplementedError()

    def run_exp(self):
        """
        runs a single expiriment trial

        @return: returns result of self.goal_data
        """
        self.init_exp()
        while self.step():
            pass
        output = self.goal_data()
        return output

    def experiments(self, trials):
        """
        runs multiple experiments, resetting scene at start of each one

        @param trials: number of experiments to run
        @return: returns list of results of self.goal_data for each trial
        """
        results = []
        for trial in range(trials):
            data = self.run_exp()
            if data is None:
                return None
            results.append(data)
        return results

    def goal_data(self):
        """
        data to return at the end of each experiment trial

        @return: can be anything, None signals error
        """
        raise NotImplementedError()

    ####################################################################################################################
    # utility functions
    ####################################################################################################################
    def set_up_agents(self, position_arrray, tolerance=.5, timeout=100):
        """
        Moves agents to specified locations
        @param position_arrray: array of R^3 numpy arrays for each blimp (must be length of num_blimps)
            if we dont care about a blimp, just use None instead of a position
        @param tolerance: tolerance of position
        @param timeout: seconds to try this before giving up
        @return:
        """
        old_command_type = self.command_type
        self.change_command_type('force')
        start = time.time()
        while time.time() - start < timeout:
            done = True
            for agent_id in range(self.num_blimps):
                goal = position_arrray[agent_id]
                if goal is None:
                    self.move_agent(agent_id, np.zeros(3))
                    continue
                pos = self.get_position(agent_id=agent_id)
                vec = goal - pos
                if np.linalg.norm(vec) <= tolerance:
                    self.move_agent(agent_id, np.zeros(3))
                else:
                    done = False
                    self.move_agent(agent_id, vec)
            if done:
                break
        for agent_id in range(self.num_blimps):
            self.move_agent(agent_id, np.zeros(3))
        self.change_command_type(old_command_type)


if __name__ == "__main__":
    test = PhysicalExperiment(cfg_paths=[create_config_file(2)], vicon=Vicon(['b2/b2']),command_type='force')
    while True:
        test.move_agent(0,np.array((0,0,-1)))
        print(test.get_velocity(0),test.get_acc(0))
        time.sleep(.1)
    test.set_up_agents([np.array((0., 0., 1.))])
