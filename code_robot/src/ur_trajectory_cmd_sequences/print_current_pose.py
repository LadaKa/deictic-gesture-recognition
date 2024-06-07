#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander
import geometry_msgs.msg


moveit_commander_args = [sys.argv[0]] 
moveit_commander.roscpp_initialize(moveit_commander_args)
manipulator = moveit_commander.MoveGroupCommander("manipulator")

rospy.init_node('print_current_pose', anonymous=True)

robot = moveit_commander.RobotCommander()


current_pose = manipulator.get_current_pose().pose
rospy.loginfo(
    "Current pose: \n"
    + "pos.: x = %.2f y = %.2f z = %.2f \n"
    + "or.:  x = %.2f y = %.2f z = %.2f w = %.2f ",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w)

rospy.loginfo(
    "%.2f %.2f %.2f %.2f %.2f %.2f %.2f ",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w)


manipulator.stop()
manipulator.clear_pose_targets()


rospy.loginfo("Finished.")

#moveit_commander.roscpp_shutdown()
