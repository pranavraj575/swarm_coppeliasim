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
4. ### Change ROS variables to connect to the Vicon laptop

   Only do this if we are connecting to the Vicon laptop, this will make ROS not work otherwise

   Run the following: (hopefully the setup we are using does not change lol)

   ```bash
   export ROS_HOSTNAME=<your computer name>.local
   export ROS_MASTER_URI=http://132.250.178.97:11311
   export ROS_IP=132.250.178.97:11311
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

3. ### Test ROS recieving Vicon messages from the system
    1. First make sure the Vicon system is up and running
       * Turn on the servers in that loud hot room
       * Turn on that Windows computer that looks like it is about to die at any moment
           * Use the Vicon account, obviously
       * open Vicon Tracker 3.9 (not sure why every possible version is installed)
       * Note: if the computer is very slow/Vicon does not show the coordinate grid, just restart the computer, not sure what thats all about
       * Calibrate
           * Option 1: go to calibration tab, hit begin calibration, and do the wand dance for like an hour
           * Better option: load saved calibration
               * hit load at the bottom of the calibration tab
               * Best one we got is named `C:\ProgramData\Vicon\Calibrations\BestCalibration` with some numbers
       * Set origin if necessary
           * Put wand on floor, the long end is the +y axis, the +z axis is in the direction of the lights, the +x axis is one of the short sticks (right hand rule)
           * Hit the origin button twice
       * Make sure objects you care about are tracking
           * Create objects in the objects tab, then check the tracking box after selecting them
           * their names should not have spaces, and should probably be useful, like `b2` for blimp 2
       * make sure screen is not paused (space to unpause after clicking the view screen)
    2. Next start brodcasting ROS messages from the connected laptop
       * `roslaunch vicon_bridge vicon.launch` should work, everything else is hypothetically in the bash.rc
    3. Finally, test if we can recieve the messages
       * Run `rostopic list` and see if the vicon topics are there
      
         should look like `\vicon\b2\b2` for an object named `b2`

       * Source both ROS and the swarm_ctrl_ws, then try `rosrun vicon_pkg msg_test -r --topic <vicon topic>`
      
         This should output the messages being brodcast by vicon
         
