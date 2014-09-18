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
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <map>
#include <fstream>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit_recorder/utils.h>

#include "moveit_recorder/trajectory_video_lookup.h"
#include "moveit_recorder/trajectory_retimer.h"

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
    ("base_dir", boost::program_options::value<std::string>(), "Directory to read the view information")
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
    std::string save_dir = vm.count("save_dir") ? vm["save_dir"].as<std::string>() : "/";
    std::string base_dir = vm.count("base_dir") ? vm["base_dir"].as<std::string>() : "/";
    std::string host = vm.count("host") ? vm["host"].as<std::string>() : "";
    size_t port = vm.count("port") ? vm["port"].as<std::size_t>() : 0;
    std::string planning_scene_topic =utils:: get_option(vm, "planning_scene_topic", "planning_scene");
    std::string display_traj_topic = utils::get_option(vm, "display_traj_topic", "/move_group/display_planned_path");
    std::string camera_placement_topic = utils::get_option(vm, "camera_topic", "/rviz/current_camera_placement");

    std::map<std::string, std::string> map_goal_to_base; // all bases files <- iterate and fill
    // read all bases files, and map their goal to the base position name
    boost::filesystem::path save_directory(save_dir);
    boost::filesystem::path base_directory(base_dir);
    boost::filesystem::directory_iterator dir_it( base_directory );
    boost::filesystem::directory_iterator end_it;
    while( dir_it != end_it )
    {
      bool is_file = boost::filesystem::is_regular_file(*dir_it);
      std::string ext = dir_it->path().extension().string();
      if (is_file && ext==".bases")
      {
        ROS_INFO("Reading bases from %s",dir_it->path().string().c_str());
        // read file and add to map
        std::ifstream file( dir_it->path().string().c_str() );
        std::string line;
        while(getline(file, line))
        {
          std::vector<std::string> tokens;
          boost::split(tokens, line, boost::is_any_of(" "), boost::token_compress_on);
          assert(tokens.size()==3);
          map_goal_to_base[tokens[1]]=tokens[2];
        }
      }
      else
        ROS_WARN("Skipping %s with extension %s (%s).", is_file?"file":"non-file",ext.c_str(),dir_it->path().string().c_str());
      ++dir_it;
    }
    ROS_INFO("Completed reading goal-to-base information. %d plans found.", (int)map_goal_to_base.size());
    
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.initializeFromWarehouse(host,port);
    
    // use this to speed up vpr
    typedef std::map<std::string, std::vector<view_controller_msgs::CameraPlacement> > BaseViewsMap;
    BaseViewsMap bvmap; //base-to-view
    
    // pubs and subs
    ros::Subscriber cam_sub = node_handle.subscribe(camera_placement_topic,1,recordViewpointCallback);
    ros::Publisher scene_pub = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic,1);
    ros::Publisher traj_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>(display_traj_topic, 1);

    utils::rostopic::waitOnSubscribersToTopic( scene_pub, planning_scene_topic );
    utils::rostopic::waitOnSubscribersToTopic( traj_pub, display_traj_topic );
    utils::rostopic::waitOnPublishersToTopic( cam_sub, camera_placement_topic );

    std::vector<view_controller_msgs::CameraPlacement> last_saved_views;
    std::vector<view_controller_msgs::CameraPlacement> current_views;
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();

    while(ros::ok() && traj_it!=video_lookup_table.end())
    {
      std::string base_name = map_goal_to_base[traj_it->second.mpr.goal_constraints[0].name];
      BaseViewsMap::iterator got = bvmap.find(base_name);
      if( got!=bvmap.end() )
      {
        //cached copy
        ROS_INFO("Found views for traj:\"%s\" with base:\"%s\"", traj_it->first.c_str(), base_name.c_str());
        for(size_t i=0; i<got->second.size(); ++i)
          video_lookup_table.put(traj_it->first, got->second[i]);
        ++traj_it;
      }
      else
      {
        //not cached
        ROS_INFO("No views found for traj:\"%s\", prompting...", traj_it->first.c_str());
        // visualize next
        scene_pub.publish(traj_it->second.ps);
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = traj_it->second.mpr.start_state;
        display_trajectory.trajectory.push_back(traj_it->second.rt);
        traj_pub.publish(display_trajectory);
        // block til done
        current_views.clear();
        int c = recorder_utils::getch();
        while(c!='n')
        {
          ros::spinOnce();
          usleep(1000);
          if(c=='a')
          {
            current_views.push_back(last_recorded_msg);
            ROS_INFO("Added %d views so far.", (int)current_views.size());
          }
          c = recorder_utils::getch();
        }
        ROS_INFO("Added %d views",(int)current_views.size());
        //cache and add
        bvmap[base_name]=current_views;
      }
    }
    video_lookup_table.saveToBag(save_dir);
    ROS_INFO("Finished grabbing views! %d Unique vantage sets.", (int)bvmap.size());
    ros::shutdown();
  }
  catch(...)
  {
    //TODO catch possible exceptions from file io and directory creation
  }
  return 0;
}
