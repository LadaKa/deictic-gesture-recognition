#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float current_x,current_y; 
bool current_pose_received = false;

void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  current_pose_received = true;
  ROS_INFO("Received");
  std::cout << current_x << "\n"; 
  std::cout << current_y << "\n"; 
  std::cout 
    << msg->pose.pose.orientation.x << " " 
    << msg->pose.pose.orientation.y << " "
    <<  msg->pose.pose.orientation.z << " " 
    << msg->pose.pose.orientation.w << "\n"; 
  
}

int main(int argc, char** argv){
  ROS_INFO("Start");
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 100,  poseCallBack);
 
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  while(!current_pose_received){
    ros::spinOnce();
    ROS_INFO("Waiting for current pose");
  }

  // Y > 0
  double left  =  M_PI/2; // 0 0 0.707107 0.707107

  // Y < 0
  double right = -M_PI/2; // 0 0 -0.707107 0.707107

  // backwards (X): M_PI: // 0 0 1 6.12323e-17

  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(M_PI/2);

  std::cout 
    << quaternion.getX() << " " 
    << quaternion.getY() << " "
    << quaternion.getZ() << " " 
    << quaternion.getW() << "\n"; 

  

  
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();


  // goal.target_pose.pose.position.x = 1.0; 
  // goal.target_pose.pose.orientation.w = 1; 

   // ok: fwd, rigth:
   goal.target_pose.pose.position.x = 1.0; 
   goal.target_pose.pose.position.y = -1.0;    //!!

   goal.target_pose.pose.orientation.z = -0.707107;
   goal.target_pose.pose.orientation.w = 0.707107;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved!");
  else
    ROS_INFO("The base failed to move for some reason");

  return 0;
}