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

#include <moveit/move_group_interface/move_group.h>

#include <moveit/warehouse/planning_scene_storage.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <view_controller_msgs/CameraPlacement.h>

#include "moveit_recorder/trajectory_retimer.h"
#include "moveit_recorder/animation_recorder.h"
#include "moveit_recorder/utils.h"

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <algorithm>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "playback");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20); // to let RVIZ come up

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("host", boost::program_options::value<std::string>(), "Host for the MongoDB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.")
    ("views",boost::program_options::value<std::string>(), "Bag file for viewpoints")
    ("camera_topic",boost::program_options::value<std::string>(), "Topic for publishing to the camera position")
    ("planning_scene_topic",boost::program_options::value<std::string>(), "Topic for publishing the planning scene for recording")
    ("display_traj_topic",boost::program_options::value<std::string>(), "Topic for publishing the trajectory for recorder")
    ("animation_status_topic",boost::program_options::value<std::string>(), "Topic for listening to the completion of the replay")
    ("save_dir",boost::program_options::value<std::string>(), "Directory for saving videos");

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
    //connect to the DB
    std::string host = vm.count("host") ? vm["host"].as<std::string>() : "";
    size_t port = vm.count("port") ? vm["port"].as<std::size_t>() : 0;
    moveit_warehouse::PlanningSceneStorage pss(host, port);
    ROS_INFO("Connected to Warehouse DB at host (%s) and port (%d)", host.c_str(), (int)port);

    // set up the storage directory
    boost::filesystem::path storage_dir( vm.count("save_dir") ? vm["save_dir"].as<std::string>() : "/tmp" );
    boost::filesystem::create_directories( storage_dir );

    // create bag file to track the associated video to the scene, query, and traj that spawned it
    boost::filesystem::path bagpath = storage_dir / "video_lookup.bag";
    rosbag::Bag bag(bagpath.string(), rosbag::bagmode::Write);
    bag.close();

    // load the viewpoints
    std::string bagfilename = vm.count("views") ? vm["views"].as<std::string>() : "";
    std::vector<view_controller_msgs::CameraPlacement> views;
    rosbag::Bag viewbag;
    viewbag.open(bagfilename, rosbag::bagmode::Read);
    std::vector<std::string> topics; topics.push_back("viewpoints");
    rosbag::View view_t(viewbag, rosbag::TopicQuery(topics));
    BOOST_FOREACH(rosbag::MessageInstance const m, view_t)
    {
      view_controller_msgs::CameraPlacement::ConstPtr i = m.instantiate<view_controller_msgs::CameraPlacement>();
      if (i != NULL)
        views.push_back(*i);
    }
    viewbag.close();
    ROS_INFO("%d views loaded",(int)views.size());

    //TODO change these to params
    std::string camera_placement_topic = utils::get_option(vm, "camera_topic", "/rviz/camera_placement");
    std::string planning_scene_topic =utils:: get_option(vm, "planning_scene_topic", "planning_scene");
    std::string display_traj_topic = utils::get_option(vm, "display_traj_topic", "/move_group/display_planned_path");
    std::string anim_status_topic = utils::get_option(vm, "animation_status_topic", "animation_status");

    AnimationRecorder recorder( camera_placement_topic,
                                planning_scene_topic,
                                display_traj_topic,
                                anim_status_topic,
                                node_handle);

    // ask the warehouse for the scenes
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    ROS_INFO("%d available scenes to display", (int)ps_names.size());
    
    // iterate over scenes
    std::vector<std::string>::iterator scene_name = ps_names.begin();
    for(; scene_name!=ps_names.end(); ++scene_name)
    {
      ROS_INFO("Retrieving scene %s", scene_name->c_str());
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, *scene_name);
      moveit_msgs::PlanningScene ps_msg = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
      
      // ask qarehosue for the queries
      std::vector<std::string> pq_names;
      pss.getPlanningQueriesNames( pq_names, *scene_name);
      ROS_INFO("%d available queries to display", (int)pq_names.size());
      
      // iterate over the queries
      std::vector<std::string>::iterator query_name = pq_names.begin();
      for(; query_name!=pq_names.end(); ++query_name)
      {
        ROS_INFO("Retrieving query %s", query_name->c_str());
        moveit_warehouse::MotionPlanRequestWithMetadata mprwm;
        pss.getPlanningQuery(mprwm, *scene_name, *query_name);
        moveit_msgs::MotionPlanRequest mpr_msg = static_cast<const moveit_msgs::MotionPlanRequest&>(*mprwm);

        // ask warehouse for stored trajectories
        std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
        pss.getPlanningResults(planning_results, *scene_name, *query_name);
        ROS_INFO("Loaded %d trajectories", (int)planning_results.size());

        // animate each trajectory
        std::vector<moveit_warehouse::RobotTrajectoryWithMetadata>::iterator traj_w_mdata = planning_results.begin();
        for(; traj_w_mdata!=planning_results.end(); ++traj_w_mdata)
        {
          moveit_msgs::RobotTrajectory rt_msg;
          rt_msg = static_cast<const moveit_msgs::RobotTrajectory&>(**traj_w_mdata);
          // retime it
          TrajectoryRetimer retimer( "robot_description", mpr_msg.group_name );
          retimer.configure(ps_msg, mpr_msg);
          bool result = retimer.retime(rt_msg);
          ROS_INFO("Retiming success? %s", result? "yes" : "no" );
            
          //date and time based filename
          boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
          std::string vid_filename = boost::posix_time::to_simple_string(now);

          // fix filename
          std::replace(vid_filename.begin(), vid_filename.end(), '-','_');
          std::replace(vid_filename.begin(), vid_filename.end(), ' ','_');
          std::replace(vid_filename.begin(), vid_filename.end(), ':','_');


          boost::filesystem::path filename( vid_filename );
          boost::filesystem::path filepath = storage_dir / filename;

          // save into lookup
          rosbag::Bag bag(bagpath.string(), rosbag::bagmode::Append);
          bag.write<moveit_msgs::PlanningScene>(filepath.string()+"_ps", ros::Time::now(), ps_msg);
          bag.write<moveit_msgs::MotionPlanRequest>(filepath.string()+"_mpr", ros::Time::now(), mpr_msg);
          bag.write<moveit_msgs::RobotTrajectory>(filepath.string()+"_rt", ros::Time::now(), rt_msg);
          
          int view_counter=0;
          std::vector<view_controller_msgs::CameraPlacement>::iterator view_msg;
          for(view_msg=views.begin(); view_msg!=views.end(); ++view_msg)
          {
            AnimationRequest req;
            
            view_msg->time_from_start = ros::Duration(0.1);
            ros::Time t_now = ros::Time::now();
            view_msg->eye.header.stamp = t_now;
            view_msg->focus.header.stamp = t_now;
            view_msg->up.header.stamp = t_now;

            req.camera_placement = *view_msg;
            req.planning_scene = ps_msg;
            req.motion_plan_request = mpr_msg;
            req.robot_trajectory = rt_msg;
            
            // same filename, counter for viewpoint
            std::string ext = boost::lexical_cast<std::string>(view_counter++) + ".ogv";
            std::string video_file = filepath.string()+ext;
            
            req.filepath = video_file;

            // save to bag
            std_msgs::String filemsg; filemsg.data = req.filepath;
            bag.write<std_msgs::String>(filepath.string(), ros::Time::now(), filemsg);
           
            recorder.record(req);
            recorder.startCapture();
            ROS_INFO("RECORDING DONE!");
          }//view
          bag.close();
        }//traj
      }//query
    }//scene
  }
  catch(mongo_ros::DbConnectException &ex)
  {
    ROS_ERROR_STREAM("Unable to connect to warehouse. If you just created the database, it could take a while for initial setup. Please try to run the benchmark again."
        << std::endl << ex.what());
  }

  ROS_INFO("Successfully performed trajectory playback");
  ros::shutdown();
  return 0;
}
