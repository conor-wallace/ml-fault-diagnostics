#!/bin/bash
xterm -e "source ~/.bashrc; env | grep ROS; python ugv_simulator.py; bash" &
xterm -e "source ~/.bashrc; env | grep ROS; python get_data.py; bash" &
