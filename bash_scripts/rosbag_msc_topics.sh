#!/bin/bash

usage="Usage: $(basename "$0") <test-name> <env (sim/lab/real)>"

if [ $# -ne 2 ]
  then
    echo $usage
    exit
fi

TEST_NAME=$1
ENV=$2
SCRIPT_DIR=$(dirname "$(realpath $0)")

OUTPUT_DIR=$SCRIPT_DIR/../out/rosbag/$ENV/$TEST_NAME
mkdir -p $OUTPUT_DIR

if [ -e $OUTPUT_DIR/*.bag ]
then
    OLD_DIR=$OUTPUT_DIR/old
    echo "Moving old bagfile into "$OLD_DIR""
    mkdir -p $OLD_DIR
    mv $OUTPUT_DIR/*.bag $OLD_DIR
fi

TIME=$(date +%Y-%m-%d-%H-%M-%S)

# The selected topics must have a supported variable in at least ROS1
# ROS2 is not especially suited for utilizing bagged files for
# analysis, and therefore the analysis is done using ROS1. This means 
# that some of the information only available in ROS2-messages, cannot 
# be used, unless bridged to ROS1  
ANAFI_OUTPUT_TOPICS="\
        /anafi/attitude \
        /anafi/battery \
        /anafi/state \
        /anafi/rpy \
        /anafi/polled_body_velocities \
        /anafi/ned_pos_from_gnss"

ANAFI_CMD_TOPICS="\
        /anafi/cmd_takeoff \
        /anafi/cmd_land \
        /anafi/cmd_emergency \
        /anafi/cmd_rpyt \
        /anafi/cmd_moveto \
        /anafi/cmd_moveby \
        /anafi/cmd_camera \
        /anafi/cmd_moveto_ned_position"

ESTIMATE_TOPICS="\
        /estimate/ekf \
        /estimate/aprilTags/num_tags_detected \
        /estimate/person_detected"

GNC_TOPICS="\
        /guidance/pure_pursuit/velocity_reference
        /guidance/desired_ned_position"

PLANNING_TOPICS="\
        /mission_controller/planning_status"

STANDARD_TOPICS="\
        $ANAFI_OUTPUT_TOPICS \
        $ANAFI_CMD_TOPICS \
        $PLANNING_TOPICS \
        $ESTIMATE_TOPICS \
        $GNC_TOPICS \
        /tf"


echo "Bagging topics" 
rosbag record -O $OUTPUT_DIR/$TIME $STANDARD_TOPICS
