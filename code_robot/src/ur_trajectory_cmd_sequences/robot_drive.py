#!/usr/bin/python3.8

# Quaternion
# [ 0, 0, 0.899, 0.437]

# Positions
# [-1.89, 16.4, -0.00143]
# [-3.62, 21.9, -0.00143]
# [-8.56 21.4, -0.00143]
# [-8.14, 17.2, -0.00143]

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):
        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], -0.00143), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position1 = {'x' : -1.89, 'y' : 16.4}
        position2 = {'x' : -3.62, 'y' : 21.9}
        position3 = {'x' : -8.56, 'y' : 21.4}
        position4 = {'x' : -8.14, 'y' : 17.2}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.899, 'r4' : 0.437}

        # rospy.loginfo("Go to (%s, %s) pose", position1['x'], position1['y'])
        # success = navigator.goto(position1, quaternion)

        # navigator = GoToPose()
        # rospy.loginfo("Go to (%s, %s) pose", position2['x'], position2['y'])
        # success = navigator.goto(position2, quaternion)

        rospy.loginfo("Go to (%s, %s) pose", position3['x'], position3['y'])
        success = navigator.goto(position3, quaternion)

        # rospy.loginfo("Go to (%s, %s) pose", position4['x'], position4['y'])
        # success = navigator.goto(position4, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")