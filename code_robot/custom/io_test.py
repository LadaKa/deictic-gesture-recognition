from rtde_io import RTDEIOInterface as RTDEIO
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

rtde_io_ = RTDEIO("192.168.0.104")
rtde_receive_ = RTDEReceive("192.168.0.104")

# How-to set and get standard and tool digital outputs. Notice that we need the
# RTDEIOInterface for setting an output and RTDEReceiveInterface for getting the state
# of an output.

if rtde_receive_.getDigitalOutState(7):
    print("Standard digital out (7) is HIGH")
else:
    print("Standard digital out (7) is LOW")

if rtde_receive_.getDigitalOutState(16):
    print("Tool digital out (16) is HIGH")
else:
    print("Tool digital out (16) is LOW")

if rtde_receive_.getDigitalOutState(7):
    print("Standard digital out (7) is HIGH")
else:
    print("Standard digital out (7) is LOW")

if rtde_receive_.getDigitalOutState(16):
    print("Tool digital out (16) is HIGH")
else:
    print("Tool digital out (16) is LOW")


