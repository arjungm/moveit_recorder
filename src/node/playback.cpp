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
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include <view_controller_msgs/CameraPlacement.h>

#include "moveit_recorder/trajectory_retimer.h"
#include "moveit_recorder/animation_recorder.h"
#include "moveit_recorder/utils.h"

#include <algorithm>

#include "moveit_recorder/trajectory_video_lookup.h"

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
    ("scene_regex",boost::program_options::value<std::string>(), "Regex for matching the scenes")
    ("query_regex",boost::program_options::value<std::string>(), "Regex for matching the queries")
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

    // track associated videos with the plans that created them
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( storage_dir.string() );

    //TODO change these to params
    std::string camera_placement_topic = utils::get_option(vm, "camera_topic", "/rviz/camera_placement");
    std::string planning_scene_topic =utils:: get_option(vm, "planning_scene_topic", "planning_scene");
    std::string display_traj_topic = utils::get_option(vm, "display_traj_topic", "/move_group/display_planned_path");
    std::string anim_status_topic = utils::get_option(vm, "animation_status_topic", "animation_status");

    boost::regex scene_regex( utils::get_option(vm, "scene_regex", ".*") );
    boost::regex query_regex( utils::get_option(vm, "query_regex", ".*") );

    AnimationRecorder recorder( camera_placement_topic,
                                planning_scene_topic,
                                display_traj_topic,
                                anim_status_topic,
                                node_handle);
    
    
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    // for each traj
    for(;traj_it!=video_lookup_table.end();++traj_it)
    {
      // match regexes
      boost::cmatch matches;
      if(!boost::regex_match( traj_it->second.ps.name.c_str(), matches, scene_regex ))
        continue;
      
      // post process trajectory (Add timing, add pauses to start and goal states)
      moveit_msgs::RobotTrajectory post_processed_rt = traj_it->second.rt;
      TrajectoryRetimer retimer( "robot_description", traj_it->second.mpr.group_name );
      retimer.configure(traj_it->second.ps, traj_it->second.mpr);
      bool result = retimer.retime(post_processed_rt);
      retimer.addTimeToStartandGoal(post_processed_rt);
      ROS_INFO("Post processing success? %s", result? "yes" : "no" );

      //make request
      AnimationRequest req;
      req.planning_scene = traj_it->second.ps;
      req.motion_plan_request = traj_it->second.mpr;
      req.robot_trajectory = post_processed_rt;

      // for each view
      std::vector<view_controller_msgs::CameraPlacement>::iterator view_it = traj_it->second.views.begin();
      size_t view_counter=0;
      for(;view_it!=traj_it->second.views.end();++view_it)
      {
        view_controller_msgs::CameraPlacement view_msg = *view_it;
        // create filename
        std::string filename = boost::str(boost::format("%s-%d.%s") % traj_it->first % ++view_counter % "ogv");
        std::string view_id = boost::str(boost::format("%s%d") % "view" % view_counter);

        //Animation request view
        view_msg.time_from_start = ros::Duration(0.001);
        ros::Time t_now = ros::Time::now();
        view_msg.eye.header.stamp = t_now;
        view_msg.focus.header.stamp = t_now;
        view_msg.up.header.stamp = t_now;
        
        // view specific
        req.camera_placement = view_msg;
        req.filepath = (storage_dir/filename).string();

        video_lookup_table.putVideoFile( traj_it->first, view_id, req.filepath );

        recorder.record(req);
        recorder.startCapture();
        ROS_INFO("Recording view #%d for trajectory id=\"%s\" complete.", (int)view_counter, traj_it->first.c_str());
      }
      ROS_INFO("Recording for trajectory id=\"%s\" complete.", traj_it->first.c_str());
    }

    /*
    // ask the warehouse for the scenes
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    ROS_INFO("%d available scenes to display", (int)ps_names.size());
    
    // iterate over scenes
    std::vector<std::string>::iterator scene_name = ps_names.begin();
    for(; scene_name!=ps_names.end(); ++scene_name)
    {
      boost::cmatch matches;
      if(!boost::regex_match( scene_name->c_str(), matches, scene_regex ))
        continue;

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
        if(!boost::regex_match( query_name->c_str(), matches, query_regex ))
          continue;
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
          // retime it and post process it
          TrajectoryRetimer retimer( "robot_description", mpr_msg.group_name );
          retimer.configure(ps_msg, mpr_msg);
          bool result = retimer.retime(rt_msg);
          ROS_INFO("Retiming success? %s", result? "yes" : "no" );
          retimer.addTimeToStartandGoal(rt_msg);
            
          //date and time based filename
          boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
          std::string time_as_string = boost::posix_time::to_simple_string(now);

          // create trajectory ID
          std::string trajectory_id = time_as_string;
          std::replace(trajectory_id.begin(), trajectory_id.end(), '/','_');
          std::replace(trajectory_id.begin(), trajectory_id.end(), '-','_');
          std::replace(trajectory_id.begin(), trajectory_id.end(), ' ','_');
          std::replace(trajectory_id.begin(), trajectory_id.end(), ':','_');

          // save into lookup
          video_lookup_table.put( trajectory_id, ps_msg );
          video_lookup_table.put( trajectory_id, mpr_msg );
          video_lookup_table.put( trajectory_id, rt_msg );
          
          int view_counter=0;
          std::vector<view_controller_msgs::CameraPlacement>::iterator view_msg;
          for(view_msg=views.begin(); view_msg!=views.end(); ++view_msg)
          {
            // create filename
            std::string filename = boost::str(boost::format("%s-%d.%s") % trajectory_id % ++view_counter % "ogv");
            std::string view_id = boost::str(boost::format("%s%d") % "view" % view_counter);

            //Animation request data
            AnimationRequest req;
            
            view_msg->time_from_start = ros::Duration(0.001);
            ros::Time t_now = ros::Time::now();
            view_msg->eye.header.stamp = t_now;
            view_msg->focus.header.stamp = t_now;
            view_msg->up.header.stamp = t_now;

            req.camera_placement = *view_msg;
            req.planning_scene = ps_msg;
            req.motion_plan_request = mpr_msg;
            req.robot_trajectory = rt_msg;
            req.filepath = (storage_dir/filename).string();
            
            video_lookup_table.putVideoFile( trajectory_id, view_id, req.filepath );
           
            recorder.record(req);
            recorder.startCapture();
            ROS_INFO("Recording Done");
          }//view
        }//traj
      }//query
    }//scene
    */
    video_lookup_table.saveToBag( (storage_dir/"video_lookup.bag").string() );
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
