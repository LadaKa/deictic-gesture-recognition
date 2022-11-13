/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */
 
#include "astra_camera/astra_driver.h"
#include <std_msgs/Empty.h>

class astra_camera_node
{

private:
  astra_wrapper::AstraDriver *drv;
  ros::NodeHandle _nh;
  ros::Subscriber sub_stop_object_detection_stream;

public:
  void stop_object_detection_stream_cb(const std_msgs::Empty::ConstPtr &msg)
  {
    // ~AstraDriver(): 
    // stop all streams of AstraDriver device
    drv->~AstraDriver();
  }

  astra_camera_node()
  {
    sub_stop_object_detection_stream = _nh.subscribe(
        "stop_object_detection_stream",
        1,
        &astra_camera_node::stop_object_detection_stream_cb,
        this);

    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    drv = new astra_wrapper::AstraDriver(n, pnh);
    ros::spin();
  }
};

int main(int argc, char **argv)
{

  ROS_INFO("astra_camera_node: Initializing ROS... ");
  ros::init(argc, argv, "astra_camera");
  astra_camera_node();

  return 0;
}
