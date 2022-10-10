#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 

# rosservice call /velocity_controller/service/enable_controller "data: false"    # Disabling the velocity-controller
rosservice call /mpc/service/enable_controller "data: false"                    # Disabling the mpc-controller
rostopic pub /anafi/cmd_land std_msgs/Empty "{}"                                # Land drone