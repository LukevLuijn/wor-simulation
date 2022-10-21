#!/bin/bash

export LC_NUMERIC="en_US.UTF-8"

# initialize ros environment
. install.setup.bash

# run nodes with launch file
ros2 launch robot_simulation robot_simulation_launch.py

exit


