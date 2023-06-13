#!/usr/bin/env bash

# Check to make sure environment variables are set
if [[ -z "${WALLCLIMB_ROOT_DIR}" ]]; then
  echo "Environment variable WALLCLIMB_ROOT_DIR is not set"
  EXIT_FLAG=1
fi
if [[ -z "${COPPELIASIM_ROOT_DIR}" ]]; then
  echo "Environment variable COPPELIASIM_ROOT_DIR is not set"
  EXIT_FLAG=1
fi
if [[ -z "${BAGFILES_ROOT_DIR}" ]]; then
  echo "Environment variable BAGFILES_ROOT_DIR is not set"
  EXIT_FLAG=1
fi
if [[ "${ROS_DOMAIN_ID}" -ne 45 ]] || [[ -z "${ROS_DOMAIN_ID}" ]]; then
  echo "Environment variable ROS_DOMAIN_ID is not set to 45 or not set. Currently[${ROS_DOMAIN_ID}]"
  EXIT_FLAG=1
fi

# Exit if any of the environment variables have issues
if [[ "${EXIT_FLAG}" -eq 1 ]]; then
  echo "Environment variables are not set. Did you modify and source 'setEnvironmentVars.sh'?"
  echo "Exiting..."
  exit $EXIT_FLAG
fi

# Make bag file directory if it doesn't already exist
mkdir -p $BAGFILES_ROOT_DIR

START_DIR=$PWD

# Here are the main variables that will change between experiments
ALTITUDES=(0.5 1 1.5 2.0)
ALTITUDES_NAMES=('05' '10' '15' '20')
SENSORS=('n' 'w')
AGENTS=(1 5 8 10 20 50 100)

# Here are some optional variables that can be adjusted before running the script
# They are used the same in every experiment

GOAL_START_XY=(10 0)
GOAL_END_XY=(-10 0)
MOVE_TIME=20
END_TIME=300

OUTPUTFILEPREFIX="BWCE_"

# If small batches are desired, you can uncomment and adjust the following,
# which will overwrite some of the above variables
# ALTITUDES=(2)
# ALTITUDES_NAMES=('20')
# SENSORS=('w')
# AGENTS=(100)


# Now we can progress through all the experiments by
# 0. Open up Coppeliasim once at beggining
# 1. Setup names and variables and print status to terminal screen
# 2. Change to bag file dir and begin recording rosbag
# 3. Run the python experiment controller script and wait to finish
# 5. Stop bag recording
# 6. Repeat again for next experimetn with differnt variables
# 7. Close Coppeliasim at very end

cd $COPPELIASIM_ROOT_DIR
# by default, run coppeliasim in a headless mode for efficiency
./coppeliaSim.sh -h &
#./home/schuler/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh -h &

# if you want to see coppeliasim actually run, comment out the above line 
#and uncomment the next line to run coppeliasim with graphics
#./coppeliaSim.sh  &

for agent in ${AGENTS[@]}; do
  for sens in ${SENSORS[@]}; do
    for alt_i in ${!ALTITUDES[@]}; do
      NUM_AGENTS=$agent
      SENSOR_TYPE=$sens
      ALTITUDE_HOLDING_HEIGHT=${ALTITUDES[$alt_i]}
      ALT_NAME=${ALTITUDES_NAMES[$alt_i]}

      ROSBAG_NAME="${OUTPUTFILEPREFIX}na-${NUM_AGENTS}_st-${SENSOR_TYPE}_mt-${MOVE_TIME}_et-${END_TIME}_alt-${ALT_NAME}"

      echo ""
      echo "---------------------------------"
      echo "Recording rosbag:$ROSBAG_NAME"

      cd $BAGFILES_ROOT_DIR
      ros2 bag record -a -o $ROSBAG_NAME &
      ROSBAG_PID=$!
      sleep 1
      cd $WALLCLIMB_ROOT_DIR
      python3 runExperiment.py -a $NUM_AGENTS -st $SENSOR_TYPE -gs ${GOAL_START_XY[0]} ${GOAL_START_XY[1]} $ALTITUDE_HOLDING_HEIGHT -ge ${GOAL_END_XY[0]} ${GOAL_END_XY[1]} $ALTITUDE_HOLDING_HEIGHT -mt $MOVE_TIME -et $END_TIME -odp ${BAGFILES_ROOT_DIR}/${ROSBAG_NAME}
      sleep 1
      kill -INT $ROSBAG_PID 
    done
  done
done

cd $START_DIR
pkill coppeliaSim
