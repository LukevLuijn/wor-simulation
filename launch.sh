#!/bin/bash

# initialize ros environment
. install.setup.bash

# run nodes with launch file
ros2 launch robot_simulation robot_simulation_launch.py

exit


