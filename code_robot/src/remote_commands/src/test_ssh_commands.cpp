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

//  TODO:
//        - add camera position to result file
//        - add filename as arg
//        - try-catch for parsing

class test_ssh_commands_node

{
public:

  struct DetectedObject { 
    int index;
    float x;
    float y;
    float z;
  };

  int selectedObjectIndex;

  float target_x;
  float target_y;

  vector<string> getWords(string line){
    vector<string> words;
    int pos = 0;
    while(pos < s.size()){
        pos = s.find(" ");
        words.push_back(s.substr(0,pos));
        s.erase(0, pos+1); 
    }
    return words;
  }


  void ParseResultFile(string filename)
  { 
    ifstream resultFile(filename); 
  
    vector<DetectedObject> detected_objects;
    string line; 
    vector<string> words;

    if (file.is_open()) { 
        line = getline(resultFile, line);
        words = getWords(line);
        float camera_x = std::stof(words[0]);
        float camera_y = std::stof(words[1]);

        int objectsCount = stoi(getline(resultFile, line));

        for (int i = 0; i < objectsCount; i++)
        {
          DetectedObject detected_object;
          line = getline(resultFile, line);
          vector<string> words = getWords(line);
          detected_object.x = std::stof(words[0]) + camera_x;
          detected_object.y = std::stof(words[1]) + camera_y;
          detected_object.z = std::stof(words[2]);
          detected_objects.push_back(detected_object);
        }
        //cout << line << endl; 
        
        selectedObjectIndex = = stoi(getline(resultFile, line));

        line = getline(resultFile, line);
        words = getWords(line);
        target_x = std::stof(words[0]);
        target_y = std::stof(words[1]);
        file.close(); 
    } 
    else { 
        ROS_INFO("Can't open file!"); 
    } 
  }

  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ssh_commands_node");

  ros::NodeHandle nh;
  ROS_INFO("Command message received!");

  test_ssh_commands_node *node = new test_ssh_commands_node();
  node->ParseResultFile("/home/lada/Desktop/result.txt");

  // TODO:

  // Navigate to selected object (modify coords!)
  // http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
  return 0;
}
