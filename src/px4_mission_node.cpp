//----------------------------------------------------------------------------------------------------------------------
// Aerial-Core UAV Manager
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include "wirispro_manager/CameraEthStreamService.h"
#include <aerialcore_common/ConfigMission.h>
#include <mavros_msgs/WaypointReached.h>
#include <mission_lib.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h> // trigger srv from std srv
// global variables
float damping;
float start_altitude;
int uav_id = 3;

// for rosbag implementation handle by Álvaro Poma:
void StartRosbag() {
  std::string id = std::to_string(uav_id);
  std::string bashscript("rosbag record -O ~/bags/uav_" + id + "_");

  char timeString[40];
  time_t t = time(0);
  struct tm tm = *localtime(&t);

  ROS_WARN("Start of ROS BAG");
  strftime(timeString, sizeof(timeString), "%Y_%m_%d_%H_%M", &tm);
  bashscript = bashscript + timeString + ".bag  -e \"/uav_" + id +
               "/mavros/(.*)\" __name:=node_bag_uav" + id + " &";
  system(bashscript.c_str());
}
void StopRosbag() {
  std::string id = std::to_string(uav_id);
  std::string bashscript = "rosnode kill node_bag_uav" + id;
  system(bashscript.c_str());
  ROS_WARN("END of ROS BAG");
}

bool sendFiles(std_srvs::SetBool::Request &req,
               std_srvs::SetBool::Response &res) {
  ROS_WARN("Init to pass bag files ");
  std::string bashscript =
      "sshpass -p 112358 rsync -ae ~/bags/ arpa@10.42.0.2:~/bags";
  system(bashscript.c_str());
  res.success = true;
  res.message = "Success";
  return true;
}
bool newMission(aerialcore_common::ConfigMission::Request &req,
                aerialcore_common::ConfigMission::Response &res,
                grvc::Mission *mission) {

  if (req.waypoint.size() < 3) {
    ROS_ERROR("Mission too small! Send a mission with at least 3 waypoints.");
    res.success = false;
    return false;
  }

  // Clear previous missions
  mission->clear();
  mission->pushClear();

  // Takeoff WP parameters:
  geometry_msgs::PoseStamped takeoff_pose;
  takeoff_pose.header.frame_id = "geo";
  takeoff_pose.pose.position.x = req.waypoint[0].latitude;
  takeoff_pose.pose.position.y = req.waypoint[0].longitude;
  takeoff_pose.pose.position.z =
      (req.waypoint[0].altitude < 60.0 ? 60.0 : req.waypoint[0].altitude);

  mission->addTakeOffWp(takeoff_pose);

  // Pass WP parameters:
  std::vector<geometry_msgs::PoseStamped> pass_poses;
  geometry_msgs::PoseStamped pass_pose;
  pass_pose.header.frame_id = "geo";

  for (auto wp = std::next(req.waypoint.begin());
       wp != std::prev(req.waypoint.end()); ++wp) {
    pass_pose.pose.position.x = wp->latitude;
    pass_pose.pose.position.y = wp->longitude;
    pass_pose.pose.position.z = wp->altitude;
    pass_poses.push_back(pass_pose);
  }

  mission->addPassWpList(pass_poses);

  // Land WP parameters:
  geometry_msgs::PoseStamped land_pose;
  land_pose.header.frame_id = "geo";
  land_pose.pose.position.x = req.waypoint.back().latitude;
  land_pose.pose.position.y = req.waypoint.back().longitude;
  land_pose.pose.position.z = req.waypoint.back().altitude;

  mission->addLandWp(land_pose);

  // Send mission to PX4
  mission->push();

  ROS_WARN("New mission sent to the UAV!");

  res.success = true;
  return true;
}

bool startStopMission(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res,
                      grvc::Mission *mission) {
  if (req.data) {
    ROS_WARN("Running mission!");
    StartRosbag();
    mission->start();
  } else {
    StopRosbag();
    mission->stop();
    ROS_WARN("Stopping mission!");
  }

  res.success = true;
  return true;
}
void wpReachedCallback(const mavros_msgs::WaypointReached &msg) {
  ROS_INFO("Wp reached: [%d]", msg->wp_seq);
  // WP_ACTION_STAY= 0,  WP_ACTION_SIMPLE_SHOT= 1,  WP_ACTION_VIDEO_START= 2,
  // WP_ACTION_VIDEO_STOP= 3,
  //                        // WP_ACTION_CRAFT_YAW = 4,  WP_ACTION_GIMBAL_PITCH
  //                        = 5
  std_srvs::Trigger srv;

  if (acommandList.data[msg->wp_seq * 10] == 1) { // needed more than one?
    // call take a picture service
    if (_capture_client.call(srv)) {
      ROS_INFO("Capture image service called");
    } else {
      ROS_ERROR("Failed to call capture image service");
    }
  }
  if (acommandList.data[msg->wp_seq * 10] == 2) { // needed more than one?
    // call start video service
    if (_start_recording_client.call(srv)) {
      ROS_INFO("Start recording service called");
    } else {
      ROS_ERROR("Failed to start recording service");
    }
  }
  if (acommandList.data[msg->wp_seq * 10] == 3) { // needed more than one?
    // call stop video service
    if (_stop_recording_client.call(srv)) {
      ROS_INFO("Stop recording service called");
    } else {
      ROS_ERROR("Failed to call the stop recording service");
    }
  }
}

int main(int _argc, char **_argv) {

  ros::init(_argc, _argv, "px4_mission_node");

  grvc::Mission mission;

  ros::NodeHandle n;

  ros::ServiceServer new_mission_srv =
      n.advertiseService<aerialcore_common::ConfigMission::Request,
                         aerialcore_common::ConfigMission::Response>(
          "mission/new", boost::bind(newMission, _1, _2, &mission));
  ros::ServiceServer start_stop_srv =
      n.advertiseService<std_srvs::SetBool::Request,
                         std_srvs::SetBool::Response>(
          "mission/start_stop",
          boost::bind(startStopMission, _1, _2, &mission));
  ros::ServiceServer service_send_bags =
      n.advertiseService("dji_control/send_bags", sendFiles);

  ros::ServiceClient _start_recording_client =
      _nh.serviceClient<std_srvs::Trigger>("/recording_start");
  ros::ServiceClient _stop_recording_client =
      _nh.serviceClient<std_srvs::Trigger>("/recording_stop");
  ros::ServiceClient _capture_client =
      _nh.serviceClient<std_srvs::Trigger>("/recording_stop");
  ros::Subscriber wp_reached_sub =
      n.subscribe("mission/reached", 1000, wpReachedCallback);

  n.getParam("px4_mission_node/damping", damping);
  n.getParam("px4_mission_node/start_altitude", start_altitude);
  n.getParam("px4_mission_node/uav_id", uav_id);

  ROS_INFO("PX4 Mission Node ready :)");

  while (ros::ok()) {
    sleep(1);
  }

  return 0;
}
