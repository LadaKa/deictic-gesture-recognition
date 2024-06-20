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

# only script name
moveit_commander_args = [sys.argv[0]] 
moveit_commander.roscpp_initialize(moveit_commander_args)

rospy.init_node('test_up_forward_target', anonymous=True)

robot = moveit_commander.RobotCommander()

manipulator = moveit_commander.MoveGroupCommander("manipulator")

rospy.loginfo("Start of sequence.")

manipulator.set_named_target("default_up")
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[1] Moved to default_up pose.")
else:
    rospy.loginfo("[1] Error when moving to default_up pose.")
    exit()

rospy.sleep(2)

manipulator.set_named_target("forward")
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[1] Moved to forward pose.")
else:
    rospy.loginfo("[1] Error when moving to forward pose.")
    exit()
    
rospy.sleep(2)

current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Current pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

manipulator.set_named_target("test_forward_1")
manipulator.go()
rospy.sleep(2)

# move down: z = z - 1
#pose_goal = geometry_msgs.msg.Pose()
#pose_goal.orientation.w = current_pose.orientation.w
#pose_goal.position.x = current_pose.position.x
#pose_goal.position.y = current_pose.position.y 
#pose_goal.position.z = current_pose.position.z 

manipulator.set_pose_target(current_pose)
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[2] Moved down to new pose.")
else:
    rospy.loginfo("[2] Error when moving down.")
    exit()

rospy.sleep(2)


rospy.loginfo("Finished.")

moveit_commander.roscpp_shutdown()
