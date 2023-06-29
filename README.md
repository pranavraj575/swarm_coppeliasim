# Blimp Coppeliasim
Used for making swarm experiments for blimps in Coppeliasim. experiments using the NEAT evolutionary algorithm are under ```\evolution```. 
```\pymaze``` contains a modified version of the [pymaze](https://github.com/jostbr/pymaze) github.

## Installation

1. Install [Coppeliasim](https://www.coppeliarobotics.com/)

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
    
    Replace ```<path to coppelia folder>``` with the path. In our case, it was ```/home/user/<me>/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04```. Unfortunately it looks like using ```~``` does not work for later build commands, so you need to put the full directory.
    ```bash
    export COPPELIASIM_ROOT_DIR="<path to coppelia folder>"
    alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"
    ```


3. Install [ROS2](https://docs.ros.org/)

    Tested with [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) on Ubuntu 20.04
     
    Make sure the following line is in the .bashrc file (with `<distro>` replaced by ROS distribution, e.g. `foxy`)
    
    ```bash
    source /opt/ros/<distro>/setup.bash
    ```

4. Set up  the [ZMQ package](https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm) into Coppeliasim
    ```bash
    /path/to/python -m pip3 install pyzmq
    /path/to/python -m pip3 install cbor
    ```
    Then add the following to your ```.bashrc```. 
    ```bash
    export PYTHONPATH="$PYTHONPATH:$COPPELIASIM_ROOT_DIR/programming/zmqRemoteApi/clients/python"
    ```

5. Clone this directory, copy all the ```.lua``` files into the correct place (replace ```<path to coppelia>``` with the path to the Coppeliasim folder). This should be run from wherever you want the repo to be.

    ```bash
    git clone https://github.com/pranavraj575/blimp_coppeliasim
    cp blimp_coppeliasim/lua/* /<path to coppelia>/lua/
    ```
    
    For some reason there is no way to tell Coppeliasim to look in different folders for ```.lua``` files.
    Not sure if this is because Coppeliasim is silly or because I am. Either way, this method works.

6. Install the ROS2 Coppelia package according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm)

    However, the tutorial sucks, so following the directions below will work
    
    * Install dependencies
      ```bash
      sudo apt update
      sudo apt-get install xsltproc
      python3 -m pip install xmlschema
      ```
    * make a ROS2 workspace (the name can probably be different)
      ```bash
      cd ~
      mkdir -p ros2_ws/src
      ```
    * clone the [sim_ros2_interface](https://github.com/CoppeliaRobotics/simExtROS2) directory and install dependencies.
      * The best way to do this is with the folder we made
          ```bash
          cp -r /<path to blimp_coppeliasim>/setup_files/sim_ros2_interface ros2_ws/src
          ```
      * However, you can try setting up according to the [tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm) like this
          ```bash
          cd ros2_ws/src
          git clone https://github.com/CoppeliaRobotics/simExtROS2
          cd sim_ros2_interface
          git checkout coppeliasim-v4.3.0-rev12
          ```
    * build the ROS2 package (note: should be run from the workspace directory)
      ```bash
      cd ~/ros2_ws
      colcon build --symlink-install
      ```

      
