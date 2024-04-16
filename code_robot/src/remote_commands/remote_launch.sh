#!/bin/bash

# https://dominoc925.blogspot.com/2020/04/setup-to-launch-ros-nodes-on-remote.html

# source /opt/ros/noetic/setup.bash
#source /home/ladak/Desktop/WS_2024/devel/setup.bash

#exec "$@"
exec rosrun remote_commands remote_commands_node
