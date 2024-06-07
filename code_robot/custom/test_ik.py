from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import Path, PathEntry
import time

ROBOT_IP = '192.168.0.104'
ROBOT_PORT = '30004'


rtde_r = RTDEReceive(ROBOT_IP)
rtde_c = RTDEControl(ROBOT_IP)
print("Start")
init_q = rtde_r.getActualQ()

init_tcp = rtde_r.getActualTCPPose()
print(init_tcp)
print(rtde_c.isPoseWithinSafetyLimits(init_tcp))

print("IK:")
print(rtde_c.getInverseKinematics(init_tcp))
target = rtde_c.getInverseKinematics(init_tcp)
print(rtde_c.isPoseWithinSafetyLimits(target))



# Stop the RTDE control script
rtde_c.stopScript()
print("end")
