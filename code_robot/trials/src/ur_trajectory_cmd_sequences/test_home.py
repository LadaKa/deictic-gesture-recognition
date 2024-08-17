#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander
import geometry_msgs.msg


#   [1] print ur home
#   [2] move to default_up and print
#   [3] move dow2n and print
#   [4] move to default_up and print

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test_home_node', anonymous=True)

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("manipulator")

rospy.loginfo("Start of sequence.")


current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Home pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

rospy.sleep(2)

#--------------------------------------------------------------------
manipulator.set_named_target("default_up")
success = manipulator.go(wait=True)


current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Default up pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

if success:
    rospy.loginfo("[1] Moved to default up pose.")
else:
    rospy.loginfo("[1] Error when moving to default up pose.")
    exit()

#--------------------------------------------------------------------

manipulator.set_named_target("forward")
success = manipulator.go(wait=True)


current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Forward pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

if success:
    rospy.loginfo("[2] Moved to forward pose.")
else:
    rospy.loginfo("[2] Error when moving to forward pose.")
    exit()

#--------------------------------------------------------------------

manipulator.set_named_target("down")
success = manipulator.go(wait=True)


current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Down pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

if success:
    rospy.loginfo("[3] Moved to down pose.")
else:
    rospy.loginfo("[3] Error when moving to down pose.")
    exit()

#--------------------------------------------------------------------
#manipulator.set_named_target("default_up")
#success = manipulator.go(wait=True)


#current_pose = manipulator.get_current_pose().pose

#rospy.loginfo(
#    "Default up pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
#    current_pose.position.x,
#    current_pose.position.y,
#    current_pose.position.z,
 #   current_pose.orientation.w)

#if success:
#    rospy.loginfo("[4] Moved to default up pose.")
#else:
#    rospy.loginfo("[4] Error when moving to default up pose.")
#    exit()

#--------------------------------------------------------------------
manipulator.stop()
manipulator.clear_pose_targets()


rospy.loginfo("Finished.")

moveit_commander.roscpp_shutdown()
