#!/usr/bin/env bash

sleep 1.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P1000S2000#1P833S2000#2P1444S2000#3P1722S2000#4P2500S2000#5P1500S2000'}" -1
echo move to cup location
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P1000S500#1P833S500#2P1444S500#3P1722S500#4P500S500#5P1500S500'}" -1
echo pickup cup
sleep 0.7
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S2000#1P1833S2000#2P1444S2000#3P722S2000#4P500S2000#5P1500S2000'}" -1
echo move with cup
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S500#1P1833S500#2P1444S500#3P722S500#4P500S500#5P1000S500'}" -1
echo rotate cup
sleep 0.7
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S500#1P1833S500#2P1444S500#3P722S500#4P500S500#5P2000S500'}" -1
echo rotate cup
sleep 0.7
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S500#1P1833S500#2P1444S500#3P722S500#4P500S500#5P1500S500'}" -1
echo rotate back
sleep 0.7
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P500S2000#1P833S2000#2P1444S2000#3P1722S2000#4P500S2000#5P1500S2000'}" -1
echo move to place location
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P500S500#1P833S500#2P1444S500#3P1722S500#4P2500S500#5P1500S500'}" -1
echo releasing cup
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P2500S750#1P1833S750#2P1444S750#3P722S750#4P2500S750#5P1500S750'}" -1
echo making a move
sleep 0.95
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P500S2000#1P833S2000#2P1444S2000#3P1722S2000#4P2500S2000#5P1500S2000'}" -1
echo moving to cup location
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P500S2000#1P833S2000#2P1444S2000#3P1722S2000#4P500S2000#5P1500S2000'}" -1
echo grabbing cup again
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P1000S2000#1P1499S2000#2P1000S2000#3P1500S2000#4P500S2000#5P1500S2000'}" -1
echo moving to drop location
sleep 2.2
ros2 topic pub --once /sim/controller/command simulation_msgs/msg/Command "{command: '#0P1000S500#1P1499S500#2P1000S500#3P1500S500#4P2500S500#5P1500S500'}" -1
echo dropping cup

