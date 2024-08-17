#!/usr/bin/python3.8


from shutil import move
import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur_before_pick_sequence', anonymous=True)

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("ur5_arm")
# set the robot to the custom default position (similar but not same as 'up')
manipulator.set_named_target("default_up")
manipulator.go()

rospy.sleep(5)

manipulator.set_named_target("camera_pose")
manipulator.go()

rospy.sleep(5)

# TODO: 
#	- use the camera to get exact coordinates of object 
# 	- set coords as target (instead of fixed "forward" and "before_pick")

manipulator.set_named_target("forward")
manipulator.go()

rospy.sleep(5)

manipulator.set_named_target("before_pick")
manipulator.go()



# TODO: 
#	- use the gripper to pick the object 
#	- lift the object


#moveit_commander.roscpp_shutdown()
