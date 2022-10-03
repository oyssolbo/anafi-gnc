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

if [[ $ENV == "sim" ]]; then
    echo "Rosbagging sim topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        /anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/link_goodput \
        /anafi/link_quality \
        /anafi/wifi_rssi \
        /anafi/battery \
        /anafi/state \
        /anafi/pose \
        /anafi/odometry \
        /anafi/cmd_takeoff \
        /anafi/cmd_land \
        /anafi/cmd_emergency \
        /anafi/cmd_rpyt \ 
        /anafi/cmd_moveto \
        /anafi/cmd_moveby \
        /anafi/cmd_camera \
        /darknet_ros/bounding_boxes \
        /estimate/dnn_cv/heading \      
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/tcv/pose \
        /tf \
elif [[ $ENV == "lab" ]]; then
    echo "Rosbagging lab topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        /anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/link_goodput \
        /anafi/link_quality \
        /anafi/wifi_rssi \
        /anafi/battery \
        /anafi/state \
        /anafi/pose \
        /anafi/odometry \
        /anafi/cmd_takeoff \
        /anafi/cmd_land \
        /anafi/cmd_emergency \
        /anafi/cmd_rpyt \ 
        /anafi/cmd_moveto \
        /anafi/cmd_moveby \
        /anafi/cmd_camera \
        /darknet_ros/bounding_boxes \
        /estimate/dnn_cv/heading \      
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/tcv/pose \
        /qualisys/Anafi/odom \
        /qualisys/Anafi/pose \
        /qualisys/Anafi/velocity \    
        /qualisys/Platform/odom \               
        /qualisys/Platform/pose \      
        /qualisys/Platform/velocity \
        /tf \
elif [[ $ENV == "real" ]]; then
    echo "Rosbagging real topics"
    rosbag record -O $OUTPUT_DIR/$TIME \
        /anafi/image \
        /anafi/attitude \
        /anafi/gnss_location \
        /anafi/height \
        /anafi/optical_flow_velocities \
        /anafi/link_goodput \
        /anafi/link_quality \
        /anafi/wifi_rssi \
        /anafi/battery \
        /anafi/state \
        /anafi/pose \
        /anafi/odometry \
        /anafi/cmd_takeoff \
        /anafi/cmd_land \
        /anafi/cmd_emergency \
        /anafi/cmd_rpyt \ 
        /anafi/cmd_moveto \
        /anafi/cmd_moveby \
        /anafi/cmd_camera \
        /darknet_ros/bounding_boxes \
        /estimate/dnn_cv/heading \      
        /estimate/dnn_cv/position \
        /estimate/ekf \
        /estimate/tcv/pose \
        /qualisys/Anafi/odom \
        /qualisys/Anafi/pose \
        /qualisys/Anafi/velocity \    
        /qualisys/Platform/odom \               
        /qualisys/Platform/pose \      
        /qualisys/Platform/velocity \
        /tf \
fi
