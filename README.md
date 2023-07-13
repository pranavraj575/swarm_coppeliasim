# Swarm Coppeliasim
Used for making swarm experiments for blimps/other in Coppeliasim. experiments using the NEAT evolutionary algorithm are under ```\evolution```. 
```\pymaze``` contains a modified version of the [pymaze](https://github.com/jostbr/pymaze) github.

## Installation

1. ### Install [Coppeliasim](https://www.coppeliarobotics.com/)

    Tested with [version 4.3.0 rev 12](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz) on Ubuntu 20.04
    
    We left the extracted folder in the ```~/Downloads``` folder.
    If you need to change this, you should update line 16 in ```/src/swarm_experiments.py```

    Example setup:
    ```bash
    cd ~/Downloads
    wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz
    tar -xvf CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz
    rm CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04.tar.xz
    ```
    
    Update your ```.bashrc``` with the following (the alias is not required, it just makes it easier to run Coppeliasim)
    
    Replace ```<path to coppelia folder>``` with the path. In our case, it was ```/home/<username>/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04```. Unfortunately it looks like using ```~``` does not work for later build commands, so you need to put the full directory.
    ```bash
    export COPPELIASIM_ROOT_DIR="<path to coppelia folder>"
    alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"
    ```


2. ### Install [ROS2](https://docs.ros.org/)

    Tested with [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) on Ubuntu 20.04
     
    Make sure the following line is in the .bashrc file (with `<distro>` replaced by ROS distribution, e.g. `foxy`)
    
    ```bash
    source /opt/ros/<distro>/setup.bash
    ```
    Remember to source the bashrc file after:
   
    ```bash
    source ~/.bashrc
    ```

3. ### Set up the [ZMQ package](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm) into Coppeliasim
    ```bash
    pip3 install pyzmq cbor
    ```
    Then add the following to your ```.bashrc```. Note: since this references COPPELIASIM_ROOT_DIR, it must be on a line after the definition made in step 1
    ```bash
    export PYTHONPATH="$PYTHONPATH:$COPPELIASIM_ROOT_DIR/programming/zmqRemoteApi/clients/python"
    ```
    Again, remember to source the bashrc file after:
   
    ```bash
    source ~/.bashrc
    ```

4. ### Set up this project
   Clone this directory, copy all the ```.lua``` files into the correct place (replace ```<path to coppelia>``` with the path to the Coppeliasim folder). This should be run from wherever you want the repo to be.

    ```bash
    git clone https://github.com/pranavraj575/swarm_coppeliasim
    cp swarm_coppeliasim/lua/* /<path to coppelia>/lua/
    cd swarm_coppeliasim
    pip3 install -e .
    ```
    
    For some reason there is no way to tell Coppeliasim to look in different folders for ```.lua``` files.
    Not sure if this is because Coppeliasim is silly or because I am. Either way, this method works.

5. ### Install the ROS2 Coppelia package "according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm)"

    However, the tutorial sucks, so following the directions below will work
    
    * Install dependencies
      ```bash
      sudo apt update
      sudo apt-get install xsltproc
      pip3 install xmlschema
      ```
    * Note: if using a conda environment, you might get an error like [this](https://github.com/colcon/colcon-ros/issues/118)

      running the following might help:
      ```bash
      conda install -c conda-forge catkin_pkg empy lark
      ```
    * make a ROS2 workspace (the name can probably be different)
      ```bash
      cd ~
      mkdir -p ros2_ws/src
      ```
    * clone the [sim_ros2_interface](https://github.com/CoppeliaRobotics/simExtROS2) directory and install dependencies.
      * The best way to do this is with the folder we made
          ```bash
          cp -r /<path to swarm_coppeliasim>/setup_files/sim_ros2_interface ros2_ws/src
          ```
      * However, you can try setting up according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm) like this
          ```bash
          cd ros2_ws/src
          git clone https://github.com/CoppeliaRobotics/simExtROS2
          cd sim_ros2_interface
          git checkout coppeliasim-v4.3.0-rev12
          ```
    * Build the ROS2 package (note: should be run from the workspace directory)
      * Note: this command is buggy, if it fails try looking at the compile instructions in the [github tutorial](https://github.com/CoppeliaRobotics/simROS2)
        ```bash
        cd ~/ros2_ws
        colcon build --symlink-install
        ```
      * The following altarnate build command works better in cases where there are a lot of compiled interfaces for some reason
        ```bash
        cd ~/ros2_ws
        sudo apt install clang
        export CXX=clang++
        colcon build --symlink-install
        ```

## Tests
* ### Coppelia installation test
  The obvious test to see if Coppeliasim is actually installed:
  ```bash
  coppelia
  ```
  This will open the coppelia simulator. Use the ```-h``` argument to test headless mode.

  Note: if you did not alias in installation step 1, you need to type in the path to the ```coppeliaSim.sh``` file
* ### ZMQ test
  Tests if the python coppelia interface works

  First open any coppelia scene with the ```coppelia``` command. Though not necessary, it might be useful to [spawn some objects](https://www.coppeliarobotics.com/helpFiles/index.html) to see the physics work.

  Then run the following file from the ```swarm_coppeliasim``` directory
  ```bash
  python3 tests/testZeroMQRemoteAPI.py
  ```
  This will just play the scene, wait a bit, then pause.

* ### ROS blimp control test
  Tests if coppelia ROS package is set up properly

  First open coppelia with the ```coppelia``` command

  Then run the following file from the ```swarm_coppeliasim``` directory
  ```bash
  python3 tests/ros_ctrl_test.py
  ```
  This test will load an empty scene, spawn a blimp, then move the blimp up, printing the state aquired repeatedly. 
  
  Mess with the file (the RECCY variable) to output graphs of the state, and mess with the test variable to change the command given

* ### Swarm control experiments test
  Tests if swarm experiments are set up properly

  Run the following file from the ```swarm_coppeliasim``` directory
  ```bash
  python3 src/swarm_experiment.py
  ```
  This test will load a scene, load a swarm of agents in different positions, and begin the simulation, giving each agent a basic command

  The simulation will run for 10 seconds, then reset with a different swarm. Currently we implemented blimps, ankis, and quadcopters

* ### Parallelism test
  Tests if parallel instances of Coppeliasim can run 

  Run the following file from the ```swarm_coppeliasim``` directory
  ```bash
  python3 tests/mult_scene_test.py
  ```

  This should open two instances of coppelia sim. A blimp will spawn in each scene, and a command of slight positive z velocity is given to one, and a slight negative z velocity to the other.
  
  Switch between the two windows and verify that there is no interference between the two blimps. One should fly straight up and the other straight down.
  
* ### Experiment/maze blimp test
  tests the module setup and is pretty cool i think

  Run the following file from the ```swarm_coppeliasim``` directory
  ```bash
  python3 src/maze_blimps.py
  ```

  This should open an instance of coppeliasim. A maze will be generated using pymaze, and the walls will be spawned into the simulation. 5 blimps will be spawned into the environment "start cell", and the simulation will start. Each blimp will start heading down towards the exit cell, probably not making it since the command is just 'go south'

  This experiment will be run twice, and the 'goal data' will be printed for each experiment at the end of the script. The output is defined in ```src/maze_blimps``` under ```AmazingBlimp.goal_data```
