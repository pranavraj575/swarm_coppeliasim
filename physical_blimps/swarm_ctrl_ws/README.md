# ROS workspace containing Vicon reciever scripts

To set up, do the following

## Installation
1. ### Install/source a ROS 1 distribution


    Tested with [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) on Ubuntu 20.04
     
    Make sure the following line is in the .bashrc file, or is run every terminal (with `<distro>` replaced by ROS distribution, e.g. `noetic`)
    
    ```bash
    source /opt/ros/<distro>/setup.bash
    ```
    Remember to source the bashrc file after:
   
    ```bash
    source ~/.bashrc
    ```
2. ### Build this workspace
    ```bash
    cd <directory of swarm_ctrl_ws>
    catkin_make
    ```
    * Note: if conda is used, this alternate build command might be needed
      `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`
3. ### Source this workspace

   Make sure the following line is in the .bashrc file, or is run every terminal
   ```bash
   source <directory of swarm_ctrl_ws>/devel/setup.bash
   ```
## Tests

1. ### Test if vicon package is correctly built/installed

  Source both ROS and the swarm_ctrl_ws (steps 1 and 3 of installation), then run the following

  ```bash
  roscd vicon_pkg
  ```

  This should take you to the `swarm_ctrl_ws/src/vicon_pk` directory, if the error is `roscd: No such package/stack 'vicon_pk'`, this workspace is not correctly built or sourced

2. ### Test ROS message sending/recieving
   Note: in all terminals opened, make sure you source both ROS and the swarm_ctrl_ws (steps 1 and 3 of installation)
   * In one terminal, run `roscore`
   * In another terminal, start sending TransformStamped messages with `rosrun vicon_pkg msg_test -s`
     
     This should immediately start printing out a ton of output of the messages being sent (since they are being logged to ROS)
   * In yet another terminal, check if messages are recieved by running `rosrun vicon_pkg msg_test -r`
  
     This should start printing the messages that the sender is sending

test with the ros messgaer hardly know her
