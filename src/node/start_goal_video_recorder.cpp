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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include "moveit_recorder/trajectory_retimer.h"
#include "moveit_recorder/animation_recorder.h"
#include "moveit_recorder/utils.h"
#include "moveit_recorder/experiment_utils.h"
#include "moveit_recorder/trajectory_video_lookup.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "start_goal_display_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("views",boost::program_options::value<std::string>(), "Bag file for viewpoints")
    ("camera_topic",boost::program_options::value<std::string>(), "Topic for publishing to the camera position")
    ("planning_scene_topic",boost::program_options::value<std::string>(), "Topic for publishing the planning scene for recording")
    ("display_traj_topic",boost::program_options::value<std::string>(), "Topic for publishing the trajectory for recorder")
    ("animation_status_topic",boost::program_options::value<std::string>(), "Topic for listening to the completion of the replay")
    ("save_dir",boost::program_options::value<std::string>(), "Directory for videos");
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
    // read the bag file to get the file names
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    boost::filesystem::path save_directory(save_dir);
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    
    // read the bag file for the viewpoints to use
    std::string view_bag_name = vm.count("views") ? vm["views"].as<std::string>() : "";
    std::vector<view_controller_msgs::CameraPlacement> views;
    utils::rosbag_storage::getViewsFromBag( view_bag_name, views );
    ROS_INFO("%d views loaded",(int)views.size());

    // construct recorder 
    std::string camera_placement_topic = utils::get_option(vm, "camera_topic", "/rviz/camera_placement");
    std::string planning_scene_topic =utils:: get_option(vm, "planning_scene_topic", "planning_scene");
    std::string display_traj_topic = utils::get_option(vm, "display_traj_topic", "/move_group/display_planned_path");
    std::string anim_status_topic = utils::get_option(vm, "animation_status_topic", "animation_status");
    
    AnimationRecorder recorder( camera_placement_topic,
                                planning_scene_topic,
                                display_traj_topic,
                                anim_status_topic,
                                node_handle);

    // iterate over the table and upload the named videos
    TrajectoryVideoLookup::iterator trajectory_it = video_lookup_table.begin();
    for( ; trajectory_it!=video_lookup_table.end(); ++trajectory_it)
    {
      std::string traj_id = trajectory_it->first;
      
      // get display trajectories
      TrajectoryRetimer retimer( "robot_description", trajectory_it->second.mpr.group_name );
      retimer.configure(trajectory_it->second.ps, trajectory_it->second.mpr);
      
      moveit_msgs::RobotTrajectory start_display = retimer.createDisplayTrajectoryForState(trajectory_it->second.rt, 0, 20);
      moveit_msgs::RobotTrajectory goal_display = retimer.createDisplayTrajectoryForState(trajectory_it->second.rt, 
                                                trajectory_it->second.rt.joint_trajectory.points.size()-1, 20);
      
      // for each view, record
      int view_counter=0;
      std::vector<view_controller_msgs::CameraPlacement>::iterator view_msg;
      for(view_msg=views.begin(); view_msg!=views.end(); ++view_msg)
      {
        // name the video
        std::string video_id = boost::str(boost::format("%s%d") % "start" % view_counter);
        std::string video_name = boost::str(boost::format("%s-%s.%s") % traj_id % video_id % "ogv");
        std::string video_file = (save_directory/video_name).string();

        // record the video
        AnimationRequest req;

        view_msg->time_from_start = ros::Duration(0.001);
        ros::Time t_now = ros::Time::now();
        view_msg->eye.header.stamp = t_now;
        view_msg->focus.header.stamp = t_now;
        view_msg->up.header.stamp = t_now;

        req.camera_placement = *view_msg;
        req.planning_scene = trajectory_it->second.ps;
        req.motion_plan_request = trajectory_it->second.mpr;
        req.robot_trajectory = start_display;
        req.filepath = video_file;

        recorder.record(req);
        recorder.startCapture();
        video_lookup_table.putVideoFile(traj_id, video_id, video_file);

        // goal state
        video_id = boost::str(boost::format("%s%d") % "goal" % view_counter);
        video_name = boost::str(boost::format("%s-%s.%s") % traj_id % video_id % "ogv");
        video_file = (save_directory/video_name).string();
        req.robot_trajectory = goal_display;
        req.filepath = video_file;
        recorder.record(req);
        recorder.startCapture();
        video_lookup_table.putVideoFile(traj_id, video_id, video_file);

      }//views
    }//traj
    video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed recording display videos.");
  ros::shutdown();

  return 0;
}

