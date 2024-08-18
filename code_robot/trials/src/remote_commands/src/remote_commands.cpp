#include <ros/ros.h>
#include "ros/console.h"
#include <std_msgs/Int32.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include <fstream> 
#include <iostream> 

#include "simple_navigation_goal.cpp"

using std::vector;
using std::string;

//  TODO:
//        - add camera position to result file
//        - add filename as arg
//        - try-catch for parsing

class remote_commands_node

{
public:

  struct DetectedObject { 
    int index;
    float x;
    float y;
    float z;
  };

  float camera_x;
  float camera_y;

  float selected_object_x;
  float selected_object_y;

  float target_x;
  float target_y;

  vector<string> getWords(string line){
    vector<string> words;
    int pos = 0;
    while(pos < line.size()){
        pos = line.find(" ");
        //std::cout << pos << "\n"; 
        words.push_back(line.substr(0,pos));
        line.erase(0, pos+1); 
    }
   // std::cout << line << "\n"; 
   // std::cout << "End" << "\n"; 
    words.push_back(line.substr(0,pos));
    return words;
  }


  void ParseResultFile(string filename)
  { 
    
    std::ifstream resultFile(filename); 
  
    vector<DetectedObject> detected_objects;
    string line; 
    vector<string> words;

    if (resultFile.is_open()) { 
        ROS_INFO("Start parsing!");
        getline(resultFile, line);
        std::cout << line << "\n"; 
        words = getWords(line);
        std::cout << words[0] << "\n"; 
        std::cout << words[1] << "\n"; 
        camera_x = std::stof(words[0]);
        camera_y = std::stof(words[1]);
        std::cout << camera_x << "\n"; 
        std::cout << camera_y << "\n"; 
        getline(resultFile, line);

        int objectsCount = std::stoi(line);
        for (int i = 0; i < objectsCount; i++)
        {
          DetectedObject detected_object;
          getline(resultFile, line);
          vector<string> words = getWords(line);
          detected_object.x = std::stof(words[0]) + camera_x;
          detected_object.y = std::stof(words[1]) + camera_y;
          detected_object.z = std::stof(words[2]);
          std::cout << detected_object.x << " " << detected_object.y << " " << detected_object.z << "\n"; 
          detected_objects.push_back(detected_object);
        }
        //cout << line << endl; 
        getline(resultFile, line);
        int selected_object_index = std::stoi(line);

        selected_object_x = detected_objects[selected_object_index].x;
        selected_object_y = detected_objects[selected_object_index].y;

        getline(resultFile, line);
        words = getWords(line);
        target_x = std::stof(words[0]);
        target_y = std::stof(words[1]);
        std::cout << target_x << "\n"; 
        std::cout << target_y << "\n"; 
        resultFile.close(); 
    } 
    else { 
        ROS_INFO("Can't open file!"); 
    } 
  }

  bool NavigateToObjects(float x_distance_from_object){
    // convert to robot coordinates, use offset distance
    float robot_goal_x = camera_x - selected_object_x - x_distance_from_object;
    float robot_goal_y = selected_object_y;
    // send robot to goal position
    simple_navigation_goal navigation_goal;
    return navigation_goal.navigate_to_goal(robot_goal_x, robot_goal_y);
  }

  bool NavigateToTargetPosition(){
    // convert to robot coordinates (robot stops on target position)
    float robot_goal_x = camera_x - target_x;
    float robot_goal_y = target_y;
    // send robot to goal position
    simple_navigation_goal navigation_goal;
    return navigation_goal.navigate_to_goal(robot_goal_x, robot_goal_y);
  }

};

int main(int argc, char **argv)
{
  float distance_from_object = 1.0;
  ros::init(argc, argv, "remote_commands_node");
 
  ros::NodeHandle nh;
  ROS_INFO("Command message received!");

  remote_commands_node *node = new remote_commands_node();
  node->ParseResultFile("/home/ladak/Desktop/result.txt");
  sleep(5);

  bool reached_object_position = node->NavigateToObjects(distance_from_object);
  if (!reached_object_position){
    ROS_INFO("Robot couldn't reach object position.");
    return -1;
  }

  sleep(5);
  system ("python /home/ladak/Desktop/WS_2024/src/remote_commands/scripts_python/reach_for_object.py");

  sleep(5);
  bool reached_target_position = node->NavigateToTargetPosition();
  if (!reached_target_position){
    ROS_INFO("Robot couldn't reach target position.");
    return -1;
  }
  
  sleep(5);
  system ("python /home/ladak/Desktop/WS_2024/src/remote_commands/scripts_python/release_object.py");

  return 0;
}
