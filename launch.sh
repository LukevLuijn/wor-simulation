#!/bin/bash

POS_Y=10
run_command ()
{
  gnome-terminal --geometry 50x7+10+${POS_Y} -- sh -c "bash -c \"ls && ${1}\" "
  POS_Y=$(($POS_Y + 200))
}

export LC_NUMERIC="en_US.UTF-8" # export to make sure
. install/setup.bash # initialize ros environment

LAUNCH_RQT="rqt --perspective-file src/robot_simulation/config/rqt_config.perspective"
LAUNCH_RVIZ="ros2 launch robot_simulation rviz.launch.py"
LAUNCH_SIM="ros2 launch robot_simulation robot.launch.py"

run_command "${LAUNCH_SIM}"
run_command "${LAUNCH_RVIZ}"
run_command "${LAUNCH_RQT}"

exit



