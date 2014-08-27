/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Willow Garage, Inc. 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Arjun Menon */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/program_options.hpp>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit_recorder/utils.h>

#include "moveit_recorder/trajectory_video_lookup.h"

static view_controller_msgs::CameraPlacement last_recorded_msg;

void recordViewpointCallback(const boost::shared_ptr<view_controller_msgs::CameraPlacement const>& msg)
{
  last_recorded_msg = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "viewpoint_recorder");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("host", boost::program_options::value<std::string>(), "Host for the MongoDB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.")
    ("planning_scene_topic",boost::program_options::value<std::string>(), "Topic for publishing the planning scene for recording")
    ("display_traj_topic",boost::program_options::value<std::string>(), "Topic for publishing the trajectory for recorder")
    ("camera_topic",boost::program_options::value<std::string>(), "Topic for publishing to the camera position")
    ("save_dir", boost::program_options::value<std::string>(), "Directory to store the recorded bagfile of viewpoints");

  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }
  try
  {
    std::string save_dir = vm.count("save_dir") ? vm["save_dir"].as<std::string>() : "/tmp/views.bag";
    std::string host = vm.count("host") ? vm["host"].as<std::string>() : "";
    size_t port = vm.count("port") ? vm["port"].as<std::size_t>() : 0;
    std::string planning_scene_topic =utils:: get_option(vm, "planning_scene_topic", "planning_scene");
    std::string display_traj_topic = utils::get_option(vm, "display_traj_topic", "/move_group/display_planned_path");
    std::string camera_placement_topic = utils::get_option(vm, "camera_topic", "/rviz/current_camera_placement");

    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.initializeFromWarehouse(host,port);
    
    // pubs and subs
    ros::Subscriber cam_sub = node_handle.subscribe(camera_placement_topic,1,recordViewpointCallback);
    ros::Publisher scene_pub = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic,1);
    ros::Publisher traj_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>(display_traj_topic, 1);
    
    // block til other nodes are ready
    utils::rostopic::waitOnSubscribersToTopic( scene_pub, planning_scene_topic );
    utils::rostopic::waitOnSubscribersToTopic( traj_pub, display_traj_topic );
    utils::rostopic::waitOnPublishersToTopic( cam_sub, camera_placement_topic );
  
    std::vector<view_controller_msgs::CameraPlacement> last_saved_views;
    std::vector<view_controller_msgs::CameraPlacement> current_views;
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    
    while(ros::ok() && traj_it!=video_lookup_table.end())
    {
      // visualize next
      scene_pub.publish(traj_it->second.ps);
      moveit_msgs::DisplayTrajectory display_trajectory;
      display_trajectory.trajectory_start = traj_it->second.mpr.start_state;
      display_trajectory.trajectory.push_back(traj_it->second.rt);
      traj_pub.publish(display_trajectory);

      ros::spinOnce();
      usleep(1000);

      // use non blocking keygrab
      int c = recorder_utils::getch();
      if(c=='n')
      {
        if(current_views.size()!=0)
          last_saved_views = current_views;
        for(int i=0; i<last_saved_views.size();++i)
          video_lookup_table.put(traj_it->first, last_saved_views[i]);
        ROS_INFO("Associating %d views to trajectory id \"%s\"", (int)last_saved_views.size(), traj_it->first.c_str());
        
        // next traj
        current_views.clear();
        traj_it++;
      }
      else if(c=='a')
      {
        ros::spinOnce();
        usleep(1000);
        current_views.push_back(last_recorded_msg);
        ROS_INFO("Adding view. %d so far", (int)current_views.size());
      }
      else if(c=='x')
      {
        video_lookup_table.saveToBag(save_dir);
        ROS_INFO("Prematurely terminated!");
        ros::shutdown();
      }
    }
    video_lookup_table.saveToBag(save_dir);
    ROS_INFO("Finished grabbing views!");
    ros::shutdown();
  }
  catch(...)
  {
    //TODO catch possible exceptions from file io and directory creation
  }
  return 0;
}
