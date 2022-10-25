#!/bin/bash

LAUNCH_RQT="rqt --perspective-file config/rqt_config.perspective"
LAUNCH_RVIZ="rviz2 --display-config config/rviz_config.rviz"

POS_Y=10
run_command ()
{
  gnome-terminal --geometry 50x7+10+${POS_Y} -- sh -c "bash -c \"ls && ${1}\" "
  POS_Y=$(($POS_Y + 200))
}

export LC_NUMERIC="en_US.UTF-8" # export to make sure
. install/setup.bash # initialize ros environment

# TODO
ros2 launch robot_simulation robot_simulation_launch.py # Launch simulation
exit
# TODO

run_command "${LAUNCH_RQT}" # Launch RQT
run_command "${LAUNCH_RVIZ}" # Launch RVIZ

ros2 launch robot_simulation robot_simulation_launch.py # Launch simulation

exit



