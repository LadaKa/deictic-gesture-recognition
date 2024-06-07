#!/usr/bin/python3.8

from shutil import move
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# movement sequence:
#   [1] up to default position
#   [2] forward
#   [3]

# only script name
moveit_commander_args = [sys.argv[0]] 
moveit_commander.roscpp_initialize(moveit_commander_args)

rospy.init_node('test_up_forward_target', anonymous=True)

robot = moveit_commander.RobotCommander()

# TODO: check args length and type
goal_x = float(sys.argv[1])
goal_y = float(sys.argv[2])
goal_z = float(sys.argv[3])
goal_w = float(sys.argv[4])

rospy.loginfo("Target: \n"+"x = %.2f y = %.2f",
    goal_x, goal_y)

manipulator = moveit_commander.MoveGroupCommander("manipulator")
eef_link = manipulator.get_end_effector_link()
print ("============ End effector: %s" % eef_link)
rospy.loginfo("Start of sequence.")

#manipulator.set_named_target("default_up")
#success = manipulator.go(wait=True)

#if success:
#    rospy.loginfo("[1] Moved to default_up pose.")
#else:
#    rospy.loginfo("[1] Error when moving to default_up pose.")
#    exit()

#rospy.sleep(2)
manipulator.clear_pose_targets()
manipulator.set_named_target("home")
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[2] Moved to forward pose.")
else:
    rospy.loginfo("[2] Error when moving to forward pose.")
    exit()
    
rospy.sleep(2)

current_pose = manipulator.get_current_pose().pose

rospy.loginfo(
    "Current pose: \n"+"x = %.2f y = %.2f z = %.2f w = %.2f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.w)

#   Current pose: 
#   x = 0.92 y = 0.03 z = 0.39 w = 0.24

#   x = 0.89 y = 0.23 z = 0.39 w = 0.23


#   Target (left):
#   x = 0.92 y = -10


# move to requested pose

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = goal_w
pose_goal.position.x = goal_x
pose_goal.position.y = goal_y
pose_goal.position.z = goal_z
manipulator.set_pose_target(pose_goal)
success = manipulator.go(wait=True)

if success:
    rospy.loginfo("[2] Moved to TARGET pose.")
else:
    rospy.loginfo("[2] Error when moving to TARGET pose.")
    exit()

manipulator.stop()
manipulator.clear_pose_targets()


rospy.loginfo("Finished.")

moveit_commander.roscpp_shutdown()
