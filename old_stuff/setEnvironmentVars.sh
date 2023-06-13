#!/usr/bin/env bash

export ROS_DOMAIN_ID=45

#Make sure to change the following to match your system
# This WALLCLIMB directory must have the python script for running the experiment
export WALLCLIMB_ROOT_DIR='/home/schuler/projects/wall-climb-simulation'
#export COPPELIASIM_ROOT_DIR='/home/schuler/bin/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04'
export COPPELIASIM_ROOT_DIR='/home/schuler/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04'
export BAGFILES_ROOT_DIR='/home/schuler/projects/wall-climb-simulation/data'

export PYTHONPATH="${PYTHONPATH}:${COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/python"
