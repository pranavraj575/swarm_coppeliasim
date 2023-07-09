-- This file contains constants that will be used by the anki vectors
ANKI_BASELINE_L = 0.047 -- m, the width between the anki treads, measured from the center of each tread
ANKI_MAX_SPEED = 0.22 -- m/s, maximum linear velocity the anki can travel
ANKI_MODEL_NAME = 'ankiVector' -- name of the anki models in the simulator, used while searching through the scene
ANKI_WHEEL_DIAMETER = 0.022 -- m, diameter of the simulated wheel, not the real wheel diameter
ANKI_VEL_CLAMP = ANKI_MAX_SPEED*2/ANKI_WHEEL_DIAMETER -- rads/s, max ang velocity based on the max lin vel
MAX_PROXIMITY_DIST = 0.3 -- m, max distance the proximity sensor can detect 
MIN_PROXIMITY_DIST = 0.03 -- m, min distance the proximity sensor can detect 
SWARM_LIST_SIGNAL = 'SWARM_LIST_SIGNAL' -- signal used to tell the swarm list model to update the list based on scene
W_HYSTERESIS = 0.00001 -- m, value used in getVelos to ignore small changes below this threshold

PUBLISH_EVERY_NTH_STEP = 2
-- value to adjust how often (how many simulation steps) each agent publishes data, set to 0 for as fast as possible
-- with 3 agents, a value of 12 achieves about 1.66 hz


DETECTION_THRESHOLD = 0.75
STOP_THRESHOLD = 0.15

TURNING_RATE = 1.0      -- rads/s
MAX_TURNING_RATE = 2.0  -- rads/s, TODO Verify this number
FORWARD_RATE = 0.1      -- m/s
MAX_FORWARD_RATE = 0.22 -- m/s

CTRL_T = 0.1 --secs

LIGHT_PERIOD = 30 --secs

WAYPOINTS={
    {-2.5,2.5},
    {-2.5,-2.5},
    {2.5,-2.5},
    {2.5,2.5}
}
NUM_WAYPOINTS=4
ACHIEVEMENT_THRESHOLD=0.35

OMEGA_DEADBAND = 0.01 -- rads, delta from orientation facing goal

DIST_DEADBAND = 0.01 -- m, delta from goalpoint, only 2d dist
STEERING_BAND = 20* 3.14/180 -- rads, band that allows forward motion if anki is facing correct orientation
