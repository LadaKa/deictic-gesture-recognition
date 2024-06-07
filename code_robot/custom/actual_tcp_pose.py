from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import datetime
import math
import os
import psutil
import sys

ROBOT_IP = '192.168.0.104'
ROBOT_PORT = '30004'

ur_cap_port = 50002

# ur_rtde realtime priorities
rt_receive_priority = 90
rt_control_priority = 85

# Parameters
vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms

flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT

rtde_r = RTDEReceive(ROBOT_IP, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(ROBOT_IP, rtde_frequency, flags, ur_cap_port, rt_control_priority)


# Move to init position using moveL
actual_tcp_pose = rtde_r.getActualTCPPose()

print(actual_tcp_pose)

#   home up
#   [-0.002049175770316799, -0.28353288646464386, 1.0007837167896299, -0.007883109902713011, -2.221343773491707, 2.2148009894963288]

#   shoulder left
#   [0.1278846394023548, -0.2835399138431044, 0.9917930747009257, 0.14556660253024017, -2.1465883037399536, 2.1413310695171646]

