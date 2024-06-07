from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import Path, PathEntry
import time

ROBOT_IP = '192.168.0.104'
ROBOT_PORT = '30004'

ur_cap_port = 50002

rtde_r = RTDEReceive(ROBOT_IP)
rtde_c = RTDEControl(ROBOT_IP)
print("Start")
init_q = rtde_r.getActualQ()



print(init_q)
# Target in the robot base
new_q = init_q[:]
new_q[1] -= 0.20
print("new Q")
print(rtde_c.isPoseWithinSafetyLimits(new_q))

# Move asynchronously in joint space to new_q, we specify asynchronous behavior by setting the async parameter to
# 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
# by the stopJ function due to the blocking behaviour.
rtde_c.moveJ(new_q, 1.05, 1.4, False)  # TRUE DOES NOTHING?
time.sleep(0.2)



# Target in the Z-Axis of the TCP
print("Tcp")
target = rtde_r.getActualTCPPose()
print(target)
print(rtde_c.isPoseWithinSafetyLimits(target))

new_q[1] += 0.20
rtde_c.moveJ(new_q, 1.05, 1.4, False)  # TRUE DOES NOTHING?
time.sleep(0.2)

#target[2] += 0.05
#print(target)
#print(rtde_c.isPoseWithinSafetyLimits(target))



# Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter to
# 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
# by the stopL function due to the blocking behaviour.
rtde_c.moveL(target,0.25, 0.5, False)
time.sleep(0.2)
# Stop the movement before it reaches target
#rtde_c.stopL(0.5)

# Move back to initial joint configuration
#rtde_c.moveJ(init_q)

# Stop the RTDE control script
rtde_c.stopScript()
print("end")
