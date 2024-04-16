#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test_forward_node', anonymous=True) 

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("manipulator")

rospy.loginfo("Start test sequence.")

#manipulator.set_named_target("default_up")
#manipulator.go()
#rospy.sleep(2)

manipulator.set_named_target("test_forward_1")
manipulator.go()
rospy.loginfo("test_forward_1.")
rospy.sleep(2)

manipulator.set_named_target("test_forward_2")
manipulator.go()
rospy.loginfo("test_forward_2")
rospy.sleep(2)

manipulator.set_named_target("test_forward_1")
manipulator.go()
rospy.loginfo("test_forward_1")
rospy.sleep(2)

manipulator.set_named_target("default_up")
manipulator.go()
rospy.loginfo("default_up")
rospy.sleep(2)

rospy.loginfo("Finished.")

moveit_commander.roscpp_shutdown()
