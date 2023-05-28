#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('reach_forward_down', anonymous=True) 

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("manipulator")
# set the robot to the home / crouched position
manipulator.set_named_target("home")
manipulator.go()

# set the robot to align to pick the bottle
manipulator.set_named_target("align")
manipulator.go()

# set the robot to go to the down grabbing position
manipulator.set_named_target("down")
manipulator.go()

# TODO: Actuate the gripper!

# lift_and_croach.py

rospy.sleep(1)
# moveit_commander.roscpp_shutdown()
