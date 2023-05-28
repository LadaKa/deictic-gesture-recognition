#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lift_and_crouch', anonymous=True) 

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("manipulator")


# set the robot to go to the lifting position 
manipulator.set_named_target("lift")
manipulator.go()

# set the robot to the home / crouched position
manipulator.set_named_target("home")
manipulator.go()

rospy.sleep(1)
moveit_commander.roscpp_shutdown()
