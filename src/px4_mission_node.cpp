//----------------------------------------------------------------------------------------------------------------------
// Aerial-Core UAV Manager
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2021 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <aerialcore_common/ConfigMission.h>
#include <mission_lib.h>
#include <std_srvs/SetBool.h>
#include <ros/ros.h>

// global variables
float damping;
float start_altitude;
int uav_id =3;

// for rosbag implementation handle by Álvaro Poma:
void StartRosbag()
{
  std::string id = std::to_string(uav_id);
  std::string bashscript ("rosbag record -O ~/bags/uav_"+ id +"_");

  char timeString[40];
  time_t t = time(0);
  struct tm tm = *localtime(&t);

  ROS_WARN("Start of ROS BAG");
  strftime(timeString, sizeof(timeString), "%Y_%m_%d_%H_%M", &tm);
  bashscript = bashscript +  timeString+ ".bag  -e \"/uav_"+ id +"/mavros/(.*)\" __name:=node_bag_uav"+id+" &";
  system( bashscript.c_str() );
}
void StopRosbag()
{
  std::string id = std::to_string(uav_id);
  std::string bashscript  = "rosnode kill node_bag_uav"+id;
  system( bashscript.c_str() );
  ROS_WARN("END of ROS BAG");
}

bool newMission(aerialcore_common::ConfigMission::Request& req, aerialcore_common::ConfigMission::Response& res, grvc::Mission* mission) {
    
    if (req.waypoint.size()<3) {
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
    takeoff_pose.pose.position.z = (req.waypoint[0].altitude < 60.0 ? 60.0 : req.waypoint[0].altitude);

    mission->addTakeOffWp(takeoff_pose);

    // Pass WP parameters:
    std::vector<geometry_msgs::PoseStamped> pass_poses;
    geometry_msgs::PoseStamped pass_pose;
    pass_pose.header.frame_id = "geo";

    for (auto wp = std::next(req.waypoint.begin()); wp != std::prev(req.waypoint.end()); ++wp) {
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

bool startStopMission(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res, grvc::Mission* mission) {
    if (req.data) {
        ROS_WARN("Running mission!");
        StartRosbag();
        mission->start();
    }
    else {
        StopRosbag();
        mission->stop();
        ROS_WARN("Stopping mission!");
    }

    res.success = true;
    return true;
}

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "px4_mission_node");
    

    grvc::Mission mission;

    ros::NodeHandle n;

    ros::ServiceServer new_mission_srv = n.advertiseService<aerialcore_common::ConfigMission::Request, aerialcore_common::ConfigMission::Response>(
        "mission/new",
        boost::bind(newMission, _1, _2, &mission)
        );
    ros::ServiceServer start_stop_srv = n.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
        "mission/start_stop",
        boost::bind(startStopMission, _1, _2, &mission)
        );
    
    
    n.getParam("px4_mission_node/damping", damping);
    n.getParam("px4_mission_node/start_altitude", start_altitude);
    n.getParam("px4_mission_node/uav_id", uav_id);
    
    ROS_INFO("PX4 Mission Node ready :)");

    while (ros::ok()) {
        sleep(1);
    }

    return 0;
}