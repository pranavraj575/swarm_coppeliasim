This readme is to provide information regarding the blimp wall climbing simulation and can be used as an example for setting up future experiment simulations.

In general, the workflow is setup in 3 layers. 
The top most layer is the meta layer, for running many experiments and adjusting variables between each run.
The top layer in this case is a bash script called autoTest.sh.
The middle layer is the experiment master layer, for running a single experiment at a time with variables provided by the top meta layer.
The middle layer in this case is a python script called runExperiment.py.
The bottom layer is the actual simulation components, the environment, models and code.
The bottom layer in this case composes the scene (wallClimbEnv.ttt), the model/models (blimpNarrowSensor.ttm and blimpWideSensor.ttm), and the code (blimp.lua, blimpHeader.lua, common.lua, rosMsg.lua).

To describe all the details, we will work from the bottom layer up in this readme.

# The Bottom Layer: Coppeliasim Details
All of these details can be largely found from familiarity with Coppeliasim. 
It is highly recommended to do the tutorials, look at the examples, and read their documentation for tips and tricks. 
It is especially noteworthy that Coppeliasim is constantly changing and new features are being added and old features become deprecated.

## Running coppeliasim
I generally assume you have installed coppeliasim and know how to run it, but there are some additional steps in order to make it work with ros2 and to specify an additional lua search path.
See another repositiory's readme for details, "ankiCoppeliasim" at http://10.42.0.42:8080/NRL_Summer_2021/ankiCoppeliasim, notably the ROS2 and Support files sections.

## Modeling
For this experiment, the blimp model was created and modeled to be as accurate as possible to real world masses and geometry. 
Physical properties regarding friction and angular dampening can be modified through the material properties portion of the object properties (Scene Object Properties > Shape > Show dynamic properties dialog >  edit material). 
These values were hand tuned to provide dynamic behavior similar to how we expect the real world blimps to behave.
Note these are different than the mass and inertia calculations that are found in the object properties. 

After defining the volumes of objects, and knowing their weights from the real world, I would generally calculate their volume density and then use Coppeliasim's button to calculate mass and inertia properties and provide the volume density so it can calculate the mass and principal moments of inertia (Scene Object Properties > Shape > Show dynamic properties dialog >  Compute mass & inertia properties for seelcted convex shapes). 
You should verify the calculated mass is approximate to the real mass (you can change the mass in the mass box if they are different).
You can double or half the principal moments of inertia as neccesary to get similar-to-real-world dynamics.

## Customization Script and control code
The base model should have a customization script attached and is responsible for actually using code we provide. 
The script lives on the model and is responsible for controlling the model it is attached to; each model has its own script.

This means anytime the script is changed, the model has to be updated, which is not ideal for making code changes.
To solve this issue, we provide the base functionality in the model's customization script, but call functions in an external file which allows us to make code changes without requiring to save and update the model all the time.
For example, we call an external function for getting state global position, and  for reading the sensor proximity data so we can add noise to the proximity/ultrasonic sensor data.
It is important to note that the external functions are called in the same execution order as the script, so the external functions are not running as a seperate controller or seperate thread. 

The scripts are excuted in an order determined by the simulator, and at certain times, (init, actuation, and sensing steps). 
It is important to have code placed in the correct spot for correct operation during simulation.

The control code (located externally from the model) is 3 PD controllers, one for the altitiude, one for the heading, and one for the distance to the goal.
These were handtuned to get behavior similar to real world observations.
These should only be tuned after already setting the masses and inertial factors as changes to those will affect the kp and kd factor values.
See the graphs model section below for more detail.

There are various global defines in the blimbHeader.lua that can be adjusted.
Note that some of these globals may not neccesarily change simulation properties of the model (such as the max and min proximity dist values, these are just there for helping to handle proximity reads in a custom external function).

## Ros2
The blimps have code in their customization script to publish their global position to a topic with their id number as an identifier.
The blimps will publish this data at every SIM_DT (usually 0.05 seconds) to the ros network, which allows us to record ros bags if we want to perform more analysis beyond the simple analysis the middle layer does.

## Coding in bouyancy and air drag
Since coppeliasim does not model lift or air drag, we can calculate this ourselves and add it to the model during simulation.

This is done by adding a force to the blimp envelope during the actuation step.
The force is calculated in an external file (as described above) so that changes can be made to these calculations without needing to save/reload model.

The bouyancy force is a force acting against gravity, and the estimated blimp mass is calcualted in blimpHeader.lua and adjusted to get a slighlty negatively bouyant blimp. 
You can check the actual simulation mass of the blmp by attaching the "center of mass" tool model from the model browser to the blimp.
The popup display is not as helpful since it only shows 2 decimal points, but you can open the customization script attached to the tool and print the total mass (you will have to click a "reinitialize" button at the top of the script editor for those changes to take affect).

The drag force is dynamic depending on the blimp velocity.
See blimHeader.lua for some details regarding how the terminal drag velocity and motor max force is used to calculate the drag force coefficient.

## The graphs
In this directory is a "graphs.ttm" model that can be attached to the blimp model in a scene to assist in tuning the PD controllers. 
After doing so, graphs will automatically open and graph altitiude, heading, and distance errors.
This can be used to visualize the step response of the blimp system and adjust the PD controllers to get desired response.

## The Scene
The scene "wallClimbEnv.ttt" is designed to have a large floor, a 3m high wall, a goal (waypoint) object.
There is also a ros2 helper tool, although I am not sure how neccesary a Coppeliasim update made it now (before you needed it in order for the models to have a ros2 network).

## the goal object
The goal object is just a sphere that is not dynamic nor collidable/reactable.
The blimps controller will try to steer them towards this goal, and will try to maintain an altitude equal to that of the goal's z position.
Therefore, by adjusting the z position of the goal you can change the altitude holding height and change the x/y to steer the blimps to different locations.

# The middle layer: python master
So now that most of the lower level details are handled and out of the way, that leaves us with the python master that actually runs the experiment.
It provides command line argument options for speciying number of agents, sensor type, goal positions and move time, simulation end time, etc. for running a single experiment.

This python file communciates to coppeliasim through ZeroMQ, see section below, especially about the pythonpath issue.

The python file is largely commented, and has an argument parser that provides command line argument flexibility and help.
To see that usuage, try "python3 runExperiment.py --help" or simply use the script incorrectly.
I encourage you to read through the code as well.

The python code does a rudimentary analysis of "succesful" blimps at the end of the simulation (counts how many are past the wall, which measn that their x position is negative) and writes the experiment details and the result of the success to a text file in the specified directory/argument.

An example usage from the command line could be "python runExperiment.py -a 20 -st n -mt 20 -et 300" while you are in this directoy. 
Note that you would need coppeliasim to already be running.
See the top level bash script for another example usage.


## External Python Client through ZeroMQ
Coppeliasim can use an external python script to interact with the simulator (start,stop,manipulate internal objects, etc)using ZeroMQ. See https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm for more detail.

Install ZeroMQ and CBOR 
- python3 -m pip install pyzmq
- python3 -m pip install cbor

Add to python path the path to Coppeliasim's ZeroMQ files, for example:

- export PYTHONPATH="$PYTHONPATH:/home/kabacinski/bin/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/zmqRemoteApi/clients/python"

- export PYTHONPATH="$PYTHONPATH:/home/schuler/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/programming/zmqRemoteApi/clients/python"


### Testing zeroMQ works
To test that ZeroMQ is set up correctly, open any scene in Coppeliasim (having dynamic objects in the scene can help verify visually that things are working correctly, such as adding a sphere and putting it to a positive z height so it would fall when the simulation starts).

Then, run the "testZeroMQRemoteAPI.py" file through "python3 testZeroMQRemoteAPI.py" at the terminal command line (make sure python path was set correctly as described above). The simulation should start, run for 3 seconds, and then end.

# The top layer, meta bash and environment variables
This is the top layer that will actually call the python master for each experiment with the autoTest.sh script.

You may need to chmod exeuctable privelige to this script "chmod +x autoTest.sh".

Before you are able to actually run the script, you will need to modify and then source the environemnet variables.
Adjust the variables in "setEnvironmentVars.sh" for your system, and then source it "source setEnvironmentVars.sh".

Now, make any adjustments to the bash script regarding which experiment details you want to change/test.
The script should be commented okay enough for that to be straightforward.

Then, simply run "./autoTest.sh" from the command line to let the simulations go to work.

# Usage
Long story short,

First, see section "Running coppeliasim" and "External Python Client through ZeroMQ" for additional install/setup items.

If you want to just run a single experiment, then open up coppeliasim and open the wallClimbEnv.ttt scene (do not press play). 
Then run the python "runExperiments.py" and provide command line arguments as neccesary (see "The middle layer: python master" section). 

If you want to run a bunch of experiments, or by uncommenting a few lines to run just a single or subset of experiments, modify setEnvironmentVar.sh, source it, and then run ./autoTest.sh (see "The top layer, meta bash and environment variables" section for more details.)


# Tristan Tutorial
RUNNING FAMILY OF EXPERIMENTS:
1. source SetEnviromentVars.sh
2. ./autoTest.sh

RUNNING ONE EXPERIMENT
1. Start Coppeliasim from Wallclimb project directory? Not sure on this       ../../Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh
2. source SetEnviromentVars.sh in another terminal window in same directory
3. python3 runExperiment.py -a 20 -st n -mt 20 -et 300


RUNNING ONE BLIMP
1. To get working to start, I copied files into Copelliasim DirectorySomehow need to change where Copellia sim looks for files? 
1. Start Coppeliasim from Wallclimb project directory? Not sure on this       ../../Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh
2. source SetEnviromentVars.sh in another terminal window in same directory
3. python3 runExperiment.py -a 20 -st n -mt 20 -et 300

