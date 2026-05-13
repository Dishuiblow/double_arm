#!/bin/bash

colcon build 

source install/setup.bash

ros2 run light_lift_arm_6dof light_lift_arm_6dof_node 

