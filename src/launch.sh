#!/bin/bash
xterm -e "source ~/.bashrc; env | grep ROS; python circle.py; bash" &
xterm -e "source ~/.bashrc; env | grep ROS; python get_data.py; bash" &
