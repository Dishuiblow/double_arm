#!/bin/bash

# 删除现有的输出文件夹
rm -rf arm_teach_data

ros2 bag record -o arm_teach_data /joint_states