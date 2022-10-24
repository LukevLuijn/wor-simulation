#!/bin/bash

RQT_CONFIG="config/rqt_config.perspective"
RVIZ_CONFIG="config/rviz_config.rviz"

export LC_NUMERIC="en_US.UTF-8"

# initialize ros environment
. install/setup.bash
# launch rqt plot for sim/cup/speed topic with saved perspective.
gnome-terminal -- sh -c "bash -c \"rqt --perspective-file ${RQT_CONFIG}\" "
# launch rviz with saved config.
gnome-terminal -- sh -c "bash -c \"rviz2 --display-config ${RVIZ_CONFIG}\" "
# run nodes with launch file
ros2 launch robot_simulation robot_simulation_launch.py # CUP node

exit


