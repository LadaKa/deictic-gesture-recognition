#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# movement sequence:
#   [1] up to default position
#   [2] forward
#   [3] up to default position

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test_forward_node', anonymous=True)

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("manipulator")

rospy.loginfo("Start of sequence.")

manipulator.set_named_target("default_up")
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[1] Moved to default pose.")
else:
    rospy.loginfo("[1] Error when moving to default pose.")
    exit()

rospy.sleep(2)
current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Current pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

current_pose.position.z = current_pose.position.z - 0.2


rospy.loginfo(
    "Current pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)
#---------------------------------------------------------------------------

manipulator.set_named_target("test_forward_1")
manipulator.go(wait=True)
rospy.sleep(2)

success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[2] Moved forward.")
else:
    rospy.loginfo("[2] Error when moving forward.")
    exit()

rospy.sleep(2)
#---------------------------------------------------------------------------

#manipulator.set_named_target("default_up")

manipulator.set_pose_target(current_pose)
success = manipulator.go(wait=True)
rospy.sleep(2)

manipulator.stop()


if success:
    rospy.loginfo("[3] Moved nearly back to default pose.")
else:
    rospy.loginfo("[3] Error when moving back to default pose.")
    exit()

rospy.loginfo("Finished.")
#---------------------------------------------------------------------------
manipulator.clear_pose_targets()
moveit_commander.roscpp_shutdown()
