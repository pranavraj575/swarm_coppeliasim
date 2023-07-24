# Using [open-blimp](https://github.com/thedancomplex/open-blimp)

To set this up 

### 1. Install [Anaconda](https://docs.anaconda.com/free/anaconda/install/index.html)

Note: if you are installing conda for the first time, you might need to go back and [set up ZMQ](https://github.com/pranavraj575/swarm_coppeliasim/tree/master#set-up-the-zmq-package-into-coppeliasim), [install the swarm coppeliasim package](https://github.com/pranavraj575/swarm_coppeliasim/tree/master#set-up-this-project),and [install the ROS2 package](https://github.com/pranavraj575/swarm_coppeliasim/tree/master#install-the-ros2-coppelia-package-according-to-the-tutorial)

### 2. Create openBlimp environment

(if using mac or windows, use the corresponding .yml file

```bash
conda env create -f setup_env_linux.yml
conda activate openBlimp
```

### 3. Install this to python (run from physical_blimps directory)

```bash
pip3 install -e .
```

### 4. Plug in the alien looking thing

You should now check your /dev/ folder to see what the USB name is

it is probably going to be ```/dev/ttyUSB0```

### 5. Turn on blimps

plug in the batteries

### 6. connection test

make sure your computer is connected to BlimpNet-2 with the same subnet as the blimps
(ip starts with 10.42.125)

try doing (replace .52 with the blimp used. If using blimp 1, .51)
```bash
ssh pi@10.42.125.52
```
the password is raspberry

### 7. config setup
change CONFIG.py as needed

### 8. configure aceess (needs to be run on every terminal)

replace the ```/dev/ttyUSB0``` with whatever the folder from part 4 is
```bash
sudo chmod 777 /dev/ttyUSB0
```

### 9. Now set up the ROS stuff for [Vicon](https://github.com/pranavraj575/swarm_coppeliasim/tree/master/physical_blimps/swarm_ctrl_ws)

