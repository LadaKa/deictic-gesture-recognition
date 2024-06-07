#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# move to target

# home up:
# pos.: x = 0.00 y = 0.19 z = 1.00 
# or.:  x = -0.71 y = 0.01 z = -0.00 w = 0.71 




# only script name
moveit_commander_args = [sys.argv[0]] 
moveit_commander.roscpp_initialize(moveit_commander_args)

rospy.init_node('move_to_target', anonymous=True)

robot = moveit_commander.RobotCommander()


# TODO: check args length and type
goal_x = float(sys.argv[1])
goal_y = float(sys.argv[2])
goal_z = float(sys.argv[3])
goal_or_x = float(sys.argv[4])
goal_or_y= float(sys.argv[5])
goal_or_z = float(sys.argv[6])
goal_or_w = float(sys.argv[7])


rospy.loginfo(
    "Target: \n"
    + "pos.: x = %.2f y = %.2f z = %.2f \n"
    + "or.:  x = %.2f y = %.2f z = %.2f w = %.2f ",
    goal_x ,
    goal_y ,
    goal_z,
    goal_or_x ,
    goal_or_y,
    goal_or_z ,
    goal_or_w )



manipulator = moveit_commander.MoveGroupCommander("manipulator")

manipulator.clear_pose_targets()
manipulator.set_goal_position_tolerance(0.05)
manipulator.set_goal_orientation_tolerance(0.001)
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

# move to requested pose

pose_goal = geometry_msgs.msg.Pose()

pose_goal.position.x = goal_x
pose_goal.position.y = goal_y
pose_goal.position.z = goal_z
pose_goal.orientation.x = goal_or_x
pose_goal.orientation.y = goal_or_y
pose_goal.orientation.z = goal_or_z
pose_goal.orientation.w = goal_or_w
manipulator.set_pose_target(pose_goal)
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("Moved to TARGET pose.")
else:
    rospy.loginfo("Error when moving to TARGET pose.")
    exit()

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

manipulator.stop()
manipulator.clear_pose_targets()


rospy.loginfo("Finished.")

#moveit_commander.roscpp_shutdown()
