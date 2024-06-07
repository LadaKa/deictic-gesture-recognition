#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# movement sequence:
#   [1] forward
#   [2] goal


# only script name
moveit_commander_args = [sys.argv[0]] 
moveit_commander.roscpp_initialize(moveit_commander_args)

rospy.init_node('forward', anonymous=True)

robot = moveit_commander.RobotCommander()

# TODO: check args length and type
goal_x = float(sys.argv[1])
goal_y = float(sys.argv[2])


manipulator = moveit_commander.MoveGroupCommander("manipulator")
manipulator.set_planning_time(10)

manipulator.clear_pose_targets()
#manipulator.set_goal_tolerance(0.5)

rospy.loginfo("Start of sequence.")

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

# move to requested pose

rospy.loginfo("Target: \n"+"x = %.2f y = %.2f",
    goal_x, goal_y)
    
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = current_pose.orientation.w
pose_goal.position.x = goal_x
pose_goal.position.y = goal_y
pose_goal.position.z = current_pose.position.z 

manipulator.set_pose_target(pose_goal)
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[1] Moved to TARGET pose.")
else:
    rospy.loginfo("[1] Error when moving to TARGET pose.")
    exit()

manipulator.stop()
manipulator.clear_pose_targets()


rospy.loginfo("Finished.")

moveit_commander.roscpp_shutdown()
