#!/usr/bin/env python3
 
import os
import sys
import argparse
import random
import math
import time
from zmqRemoteApi import RemoteAPIClient

# Function to move an object until there is no collision. For now it just steps the object in a single direction 
# until there is no collisions detected. Not the smartest and may be prone to errors, but fine for now
def moveObjectUntilNoCollision(agentHandle):
    moveCountLimit = 50
    moveCount = 0 # how many times did we try to find a new position, used as a watchdog
    moveOffset = 0.1 #m, how much distance to move to try again
    position = sim.getObjectPosition(agentHandle, -1)
    collisionResult, _ = checkForCollisionWrapper(agentHandle)
    while(collisionResult):
        if(moveCount > moveCountLimit):
            print("Warning: Failed to move the object with handle [{}] to new location within specified limit. Trying up.".format(agentHandle))
            break
        position[0] = position[0] + moveOffset
        sim.setObjectPosition(agentHandle, -1, position)
        collisionResult, _ = checkForCollisionWrapper(agentHandle)
        moveCount = moveCount + 1
    # If we are here with no collisions, we succesful found a spot with no collision
    if not collisionResult:
        return

    # here we failed, so lets try again but no move upwards
    moveUpCountLimit = 50
    moveUpCount = 0 # how many times did we try to find a new position, used as a watchdog
    moveUpOffset = 0.1 #m, how much distance to move to try again
    position = sim.getObjectPosition(agentHandle, -1)
    collisionResult, _ = checkForCollisionWrapper(agentHandle)
    while(collisionResult):
        if(moveUpCount > moveUpCountLimit):
            print("Failed to move the object with handle [{}] to new location within specified limit. \nExiting...".format(agentHandle))
            exit(1)
        position[2] = position[2] + moveUpOffset
        sim.setObjectPosition(agentHandle, -1, position)
        collisionResult, _ = checkForCollisionWrapper(agentHandle)
        moveUpCount = moveUpCount + 1
    # print("isColliding:[{}] Number of moves:[{}]".format(collisionResult, moveCount))
    

# Wrapper for calling simulator collison check since it doesn't return consistent argument type
# Returns a result and a list
#   collisionResult : bool, true or false if there is a collision
#   collidingObjectHandles: list, list of colliding object handles
def checkForCollisionWrapper(agentHandle):
    collisionResult = None
    collidingObjectHandles = None
    result = sim.checkCollision(agentHandle, sim.handle_all)
    if type(result) is tuple:
        collisionResult = result[0]
        collidingObjectHandles = result[1]
    else:
        collisionResult = result
    collisionResult = bool(collisionResult)
    return collisionResult, collidingObjectHandles

#  Function to calculate the average distance from each blimp to the goal position. Currently unused.
def calculateAvgDistanceFromGoal(agentHandles, goalPosition):
    totalNum = len(agentHandles)
    totalDist = 0
    for agentHandle in agentHandles:
        x, y, z = sim.getObjectPosition(agentHandle, -1)
        gx, gy, gz = goalPosition
        dist = math.sqrt((x-gx)**2+(y-gy)**2+(z-gz)**2)
        totalDist = totalDist + dist
    return totalDist/totalNum

# Calculate the number of succesful blimps. In this case, they are if they passed the wall.
# Since the wall is located at the origin and along the y axis, if a blimp has a position with a negative x,
# the blimp has passed the wall
def calculateSuccesfulBlimps(agentHandles):
    count = 0
    for agentHandle in agentHandles:
        x, y, z = sim.getObjectPosition(agentHandle, -1)
        if x < 0:
            count = count + 1
    return count

# Function that will find an unused Exp ID number that will be used in naming the output file
# Requires the output file prefix for template matching, and requires the id number follows the prefix and is 
# terminated with an underscore '_' after the id number
# Finds the highest existing number in the specified directory (defaults to current dir) and adds 1
# Used to help avoid overwriting existing files
def findUnusedExpID(outputFilePrefix, outputDir=os.getcwd()):
    highestExpNum = 0
    for x in os.listdir(outputDir):
        if x.startswith(outputFilePrefix) and x.endswith(".txt"):
            firstUnder = x.find("_")
            secondUnder = x.find("_", firstUnder+1)
            if firstUnder == -1 or secondUnder == -1:
                continue
            expNum = x[firstUnder+1:secondUnder]
            try:
                expNum = int(expNum)
            except:
                continue
            if(expNum >= highestExpNum):
                highestExpNum = expNum + 1
    return highestExpNum

if __name__ == "__main__":
    # Values used as default conditions for running the experiment.
    # Can be changed by using command line arguments. See the Argument Parser
    goalStart = [10.0, 0.0, 1.0] # x,y,z
    goalEnd = [-10.0, 0.0, 1.0] # x,y,z
    startZone = {'xmin':5, 'xmax':15,
                'ymin':-5, 'ymax':5,
                'zmin':0.5, 'zmax':2.0,
                }
    DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

    outputFilePrefix = "blimpWallClimbExp_"

    # Argument parser for running options and help descriptions
    parser = argparse.ArgumentParser(description = "Script used to run blimp wall climbing experiment")
    parser.add_argument("-a","--agents", type=int, required=True, metavar=('numAgents'), 
                help="Specify number of agents to be spawned")
    parser.add_argument("-st","--sensortype", type=str, required=True, choices=['w','wide','n','narrow'],
                help="Specify a wide ('w' or 'wide') or narrow ('n' or 'narrow') sensor type.")
    parser.add_argument("-nmp","--narrowmodelpath", type=str, metavar=('path'), default=DIR+'/old_models/blimpNarrowSensor.ttm',
                help="Specify path to the narrow model. Default is '"+DIR+"/old_models/blimpNarrowSensor.ttm'. \
                    Can also accept 'blimpNarrowSensor.ttm' if running from within the correct dir.")
    parser.add_argument("-wmp","--widemodelpath", type=str, metavar=('path'), default=DIR+'/old_models/blimpWideSensor.ttm',
                help="Specify path to the narrow model. Default is '"+DIR+"/old_models/blimpWideSensor.ttm'. \
                    Can also accept 'blimpWideSensor.ttm' if running from within the correct dir.")
    parser.add_argument("-sp","--scenepath", type=str, metavar=('path'), default=DIR+'/scenes/wallClimbEnv.ttt',
                help="Specify path to the simulation scene. Default is '"+DIR+"/scenes/wallClimbEnv.ttt'. \
                    Can also accept 'wallClimbEnv.ttt' if running from within the correct dir.")
    parser.add_argument("-gs","--goalstart", type=float, nargs=3, default=goalStart, metavar=('x', 'y', 'z'),
                help="Specify the x y z coordinates for the goal's beginning position. \
                    Note, the z coordinate may be used as the altitude hold height. \
                    Default is 10.0 0.0. 1.0")
    parser.add_argument("-ge","--goalend", type=float, nargs=3, default=goalEnd, metavar=('x', 'y', 'z'),
                help="Specify the x y z coordinates for the goal's end position. \
                    Note, the z coordinate may be used as the altitude hold height. \
                    Default is -10.0 0.0. 1.0")
    parser.add_argument("-sz","--spawnzone", type=float, nargs=6, default=[5,15,-5,5,0.5,2], 
                metavar=('xmin','xmax', 'ymin', 'ymax', 'zmin', 'zmax'),
                help="Specify the xmin,xmax, ymin, ymax, zmin, zmax coordinates for the zone the blimps can spawn into. \
                    The program will attempt to spawn the blimps in a random location within this area,and try \
                    some basic collision avoidance. Default is 5 15 -5 5 0.5 2")
    parser.add_argument("-mt","--movetime", type=float, default=20.0, metavar=('secs'),
                help="Specify the time in seconds the goal will move from its start position to the end position. Default 20 seconds")
    parser.add_argument("-et","--endtime", type=float, default=150.0, metavar=('secs'),
                help="Specify the time in seconds when the simulation will stop. Default 150 seconds")    
    parser.add_argument("-p","--pause",action="store_true",
                help="Specify this parameter to pause at the end of simulation instead of stop. Useful if desired hand count at end.")
    parser.add_argument("-odp","--outputdirectorypath", type=str, metavar=('path'), default=DIR+'/old_stuff/data',
                help="Specify path to the output directory for the summary file to be written within. Default is '"+DIR+"/old_stuff/data")
    
    # Parse the arguments and assign into variables
    args = parser.parse_args()
    agentHandles = []
    numAgents = args.agents
    sensorType = args.sensortype
    narrowModelPath = os.path.abspath(os.path.expanduser(args.narrowmodelpath))
    wideModelPath = os.path.abspath(os.path.expanduser(args.widemodelpath))
    sceneNamePath = os.path.abspath(os.path.expanduser(args.scenepath))
    outputDirectoryPath = os.path.abspath(os.path.expanduser(args.outputdirectorypath))
    goalStart = args.goalstart
    goalEnd = args.goalend
    startZone["xmin"] = args.spawnzone[0]
    startZone["xmax"] = args.spawnzone[1]
    startZone["ymin"] = args.spawnzone[2]
    startZone["ymax"] = args.spawnzone[3]
    startZone["zmin"] = args.spawnzone[4]
    startZone["zmax"] = args.spawnzone[5]
    moveTime = args.movetime
    endTime = args.endtime
    doPauseAtEnd = args.pause

    # Verify all files for simulation actually exists
    files = [narrowModelPath, wideModelPath, sceneNamePath]
    for file in files:
        try:
            assert os.path.exists(file)
        except:
            print('Failed at finding [' + file + '].\nExiting...')
            exit(1)
    if not  os.path.isdir(outputDirectoryPath):
        raise Exception('no  path')

    #####################
    # Now that argument parsing has been completed, begin setting up the simulation
    # Setup remote python clinet to interface with coppeliasim and load scene
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    sim.stopSimulation()
    time.sleep(1)

    sim.loadScene(sceneNamePath)
    time.sleep(1) # small delay to let coppeliasim perform last call
    
    #Check for goal existence, and position it accordingly, exit on failure
    goalHandle = sim.getObject("/Goal", {"noError":True})
    if goalHandle == -1 :
        print("Error: No goal object in scene, \nExiting...")
        exit(1)
    sim.setObjectPosition(goalHandle, -1, goalStart)

    # Load in the blimps, placing randomly in the start area and checking for collisions
    modelToLoad = narrowModelPath if sensorType == 'n' or sensorType == 'narrow' else wideModelPath
    for agent in range(numAgents):
        agentHandle = sim.loadModel(modelToLoad)
        agentHandles.append(agentHandle) # Save loaded models' handles for analysis later
        xPos = round(random.uniform(startZone["xmin"],startZone["xmax"]),2)
        yPos = round(random.uniform(startZone["ymin"],startZone["ymax"]),2)
        zPos = round(random.uniform(startZone["zmin"],startZone["zmax"]),2)
        sim.setObjectPosition(agentHandle, -1, [xPos, yPos, zPos])
        
        collisionResult,collidingObjectHandles = checkForCollisionWrapper(agentHandle)
        if collisionResult:
            # There was a collision, we should move so we do not collide
            moveObjectUntilNoCollision(agentHandle)
    
    ##################
    # Now we are setup, so we can begin the simulation
    print("STARTING SIM")
    #sim.setArrayParameter(sim.arrayparam_gravity, [0., 0., -.01])
    sim.startSimulation()

    # Wait until moving goal to other side of wall, giving time for agents to move towards first waypoint
    while (t := sim.getSimulationTime()) < moveTime:
        # Small sleep so we aren't constantly checking the time
        #for handle in agentHandles:
        #    sim.addForce(handle,[0,0,0],[0,10,0])
        time.sleep(0.2)

    # Time to move the goal to the other side of the wall
    sim.setObjectPosition(goalHandle, -1, goalEnd)

    # Now we wait for the agents to try and climb the wall until time to end the simulation
    while (t := sim.getSimulationTime()) < endTime:
        time.sleep(0.2)

    # We pause the simulation first, giving us a change to do some analyis before we completly stop the sim
    sim.pauseSimulation()
    time.sleep(1) # small delay to let coppeliasim perform last call

    # Find a unique file id number that is different than any other existing files in the directory
    # to avoid overwriting existing files
    fileID = findUnusedExpID(outputFilePrefix, outputDirectoryPath)

    # Find the number of succesful blimps, in this case those that got over the wall
    numSuccesfulBlimps = calculateSuccesfulBlimps(agentHandles)

    # Create the output file name
    outputFileName = outputFilePrefix + \
            "{}_na-{}_st-{}_mt-{}_et-{}_nsucc-{}.txt".format(fileID, numAgents,
            "n" if sensorType == 'n' or sensorType == 'narrow' else "w",
            int(moveTime), int(endTime), numSuccesfulBlimps)

    # Open and write to the output file with all experiment relevant information
    outputFileFullPath = os.path.join(outputDirectoryPath,outputFileName)
    with open(outputFileFullPath, 'w') as f:
        lines = []
        experimentHeaderStr = "Blimp Wall Climbing Experiment"
        numberOfAgentsStr = "Number Blimps: " + str(numAgents)
        blimpTypeStr = "Blimp Ultrasonic: Narrow" if sensorType == 'n' or sensorType == 'narrow' else "blimp Ultrasonic: Wide"
        goalStartStr = "Goal Start: " + str(goalStart)
        goalEndStr = "Goal End: " + str(goalEnd)
        startZoneStr = "Start Zone [-x,+x,-y,+y,-z,+z]: " + str(args.spawnzone)
        moveTimeStr = "Goal Move Time: " + str(moveTime) + " s"
        endTimeStr = "Simulation End Time: " + str(endTime) + " s"
        numSuccesfulBlimpsStr = "\nNumber of blimps that made it over wall: " + str(numSuccesfulBlimps)

        lines.append(experimentHeaderStr + "\n")
        lines.append(numberOfAgentsStr + "\n")
        lines.append(blimpTypeStr + "\n")
        lines.append(goalStartStr + "\n") 
        lines.append(goalEndStr + "\n")
        lines.append(startZoneStr + "\n")
        lines.append(moveTimeStr + "\n")
        lines.append(endTimeStr + "\n")
        lines.append(numSuccesfulBlimpsStr + "\n")
        f.writelines(lines)

    # Stop the simulation completly, or otherwise leave paused if correct flag was specified on the command line
    if not doPauseAtEnd:
        sim.stopSimulation()

    print("Done with experiment, results written to file {}".format(outputFileFullPath))
 
