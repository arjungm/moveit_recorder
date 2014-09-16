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

#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_common.h>
#include "moveit_recorder/scene_robot_control.h"
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <fstream>

SceneRobotControl::SceneRobotControl(ros::NodeHandle nh, 
    const std::string& planning_scene_topic,
    const std::string& from_marker_topic,
    const std::string& from_marker_pose_topic,
    const std::string& to_marker_topic,
    const std::string& query_save_location)
: node_handle_(nh), 
  is_scene_initialized_(false), 
  planning_scene_topic_(planning_scene_topic),
  from_marker_topic_(from_marker_topic),
  from_marker_pose_topic_(from_marker_pose_topic),
  to_marker_topic_(to_marker_topic),
  query_num_(0),
  base_num_(1),
  query_file_location_(query_save_location)
{
  // publishers
  planning_scene_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>(planning_scene_topic_,1);
  robot_state_publisher_ =  node_handle_.advertise<moveit_msgs::RobotState>(to_marker_topic_, 1);

  // subscribers
  planning_scene_subscriber_ = node_handle_.subscribe(planning_scene_topic_, 1,
      &SceneRobotControl::planningSceneCallback, this);
  robot_state_subscriber_ = node_handle_.subscribe(from_marker_topic_, 1,
      &SceneRobotControl::markerRobotStateCallback, this);
  robot_pose_subscriber_ = node_handle_.subscribe(from_marker_pose_topic_, 1,
      &SceneRobotControl::markerRobotPoseCallback, this);

  // collision checking
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description")); // TODO is param?
  robot_model_ = robot_model_loader_->getModel();
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  planning_scene_ = boost::make_shared<planning_scene::PlanningScene>(robot_model_);
}

SceneRobotControl::~SceneRobotControl()
{
  planning_scene_.reset();
  robot_state_.reset();
  robot_model_.reset();
}

void SceneRobotControl::markerRobotPoseCallback(const boost::shared_ptr<geometry_msgs::Pose const>& msg)
{
  current_pose_ = *msg;
}

void SceneRobotControl::planningSceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
{
  if(!is_scene_initialized_)
  {
    planning_scene_->usePlanningSceneMsg(*msg);
    current_scene_ = *msg;
    current_state_ = msg->robot_state;
    is_scene_initialized_ = true;
    planning_scene_publisher_.publish(current_scene_); // to rviz & move group
    robot_state_publisher_.publish(current_state_); // to interactive robot
    query_num_ = 0;
  }
}

void SceneRobotControl::markerRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg)
{
  current_state_ = *msg;
  current_scene_.robot_state = *msg;
  robot_state::robotStateMsgToRobotState(*msg, *robot_state_);
  planning_scene_publisher_.publish(current_scene_); // to rviz & move group
}

void SceneRobotControl::writePositionsToFile(const std::string& filepath, 
                                             const std::string& scene_name,
                                             const std::vector<std::string>& names,
                                             const std::vector<moveit_msgs::RobotState>& p)
{
  double roll, pitch, yaw;
  std::ofstream file;
  file.open(filepath.c_str());
  file << scene_name << std::endl;
  file << "start" << std::endl;
  file << names.size() << std::endl;
  for(size_t i=0; i<names.size(); i++)
  {
    file << names[i] << std::endl;
    for(size_t j=0; j<p[i].joint_state.name.size(); j++)
      file << p[i].joint_state.name[j] << " = " << p[i].joint_state.position[j] << std::endl;
    
    // TODO this is hard coded... want to get the world joint better
    sensor_msgs::MultiDOFJointState mdjs = p[i].multi_dof_joint_state;
    file << mdjs.joint_names[0] << "/x = " << mdjs.transforms[0].translation.x << std::endl;
    file << mdjs.joint_names[0] << "/y = " << mdjs.transforms[0].translation.y << std::endl;
    getRPY(mdjs.transforms[0], roll, pitch, yaw);
    file << mdjs.joint_names[0] << "/theta = " << yaw << std::endl;
    file << "." << std::endl;
  }
  file.close();
}

void SceneRobotControl::writePosesToFile(const std::string& filepath,
                                         const std::string& scene_name,
                                         const std::vector<std::string>& names,
                                         const std::vector<geometry_msgs::Pose>& poses)
{
  double roll, pitch, yaw;
  std::ofstream file;
  file.open(filepath.c_str(), std::ios::out | std::ios::app);
  file << "goal" << std::endl;
  file << names.size() << std::endl;
  for(size_t i=0; i<names.size(); i++)
  {
    file << "link_constraint" << std::endl;
    file << names[i] << std::endl;
    file << "r_wrist_roll_link" << std::endl; // TODO read this from somewhere...
    file << "xyz " << poses[i].position.x << " ";
    file << poses[i].position.y << " ";
    file << poses[i].position.z << std::endl;
    getRPY(poses[i], roll, pitch, yaw);
    //file << "rpy " << roll << " " << pitch << " " << yaw << std::endl;
    file << "quat " << poses[i].orientation.w << " "
                    << poses[i].orientation.x << " "
                    << poses[i].orientation.y << " "
                    << poses[i].orientation.z << std::endl;
    file << "." << std::endl;
  }
  file.close();
}
      
      
void SceneRobotControl::writeBasesToFile( const std::string& filepath,
                                          const std::string& scene_name,
                                          const std::vector<std::string>& position_names,
                                          const std::vector<std::string>& pose_names,
                                          const std::vector<std::string>& base_names)
{
  std::ofstream file;
  file.open(filepath.c_str(), std::ios::out | std::ios::app);
  
  assert( position_names.size() == base_names.size() );

  for(size_t i=0; i<position_names.size(); ++i)
    file << boost::str(boost::format( "%s %s %s\n" ) % position_names[i] % pose_names[i] % base_names[i]);

  file.close();
}

void SceneRobotControl::getControlMessage(int dir)
{
  switch(dir)
  {
    case 'o':
    {
      ROS_INFO("[Output]");
      for(int i=0; i<current_state_.joint_state.name.size();++i)
        ROS_INFO("[%35s] %6.2f", current_state_.joint_state.name[i].c_str(), 
                                current_state_.joint_state.position[i]);

      double roll, pitch, yaw;
      getRPY(current_state_.multi_dof_joint_state.transforms[0], roll, pitch, yaw);
      ROS_INFO("x:%4.2f y:%4.2f z:%4.2f r:%4.2f p:%4.2f y:%4.2f",
          current_state_.multi_dof_joint_state.transforms[0].translation.x,
          current_state_.multi_dof_joint_state.transforms[0].translation.y,
          current_state_.multi_dof_joint_state.transforms[0].translation.z,
          roll,
          pitch,
          yaw);

      getRPY(current_pose_, roll, pitch, yaw);
      ROS_INFO("x:%4.2f y:%4.2f z:%4.2f r:%4.2f p:%4.2f y:%4.2f",
                current_pose_.position.x,
                current_pose_.position.y,
                current_pose_.position.z,
                roll,
                pitch,
                yaw);
      break;
    }
    case 's':
    {
      ROS_INFO("[Save] %d joint positions and Pose saved so far", (int)(2*(++query_num_)));
      
      query_position_names_.push_back( boost::str(boost::format("%s.%s.%d") % current_scene_.name % "start" % query_num_) );
      query_positions_.push_back(current_state_);

      query_6dofpose_names_.push_back( boost::str(boost::format("%s.%s.%d") % current_scene_.name % "goal" % query_num_) );
      query_6dofposes_.push_back(current_pose_);

      query_basepose_names_.push_back( boost::str(boost::format("%s.%s.%d") % current_scene_.name % "base" % base_num_) );

      break;
    }
    case 'c':
    {
      ROS_INFO("[Collision] Checking for collisions on the whole robot");
      collision_detection::CollisionRequest req; req.contacts=true;
      collision_detection::CollisionResult res;
      planning_scene_->checkCollision(req, res, *robot_state_, planning_scene_->getAllowedCollisionMatrix());
      if(res.collision)
      {
        collision_detection::CollisionResult::ContactMap::iterator collision_pair = res.contacts.begin();
        for(; collision_pair!=res.contacts.end(); ++collision_pair)
          ROS_WARN("Collision detected between %s and %s", 
              collision_pair->first.first.c_str(), 
              collision_pair->first.second.c_str());
      }
      else
        ROS_INFO("Collision free");
      break;
    }
    case 'w':
    {
      boost::filesystem::path query_filepath( query_file_location_ );
      query_filepath = query_filepath / current_scene_.name;
      std::string query_file = boost::str( boost::format( "%s.queries" ) % query_filepath.string() ) ;
      std::string bases_file = boost::str( boost::format( "%s.bases" ) % query_filepath.string() ) ;
      ROS_INFO("[Write] Writing queries to file: %s", query_file.c_str());
      
      // write out to file the query locations (this saves the joint positions and poses for planning)
      writePositionsToFile( query_file, current_scene_.name, query_position_names_, query_positions_);
      writePosesToFile(     query_file, current_scene_.name, query_6dofpose_names_, query_6dofposes_);
      writeBasesToFile(     bases_file, current_scene_.name, query_position_names_, query_6dofpose_names_, query_basepose_names_);
      break;
    }
    case 'r':
    {
      ROS_INFO("[Reset] Scene is reset");
      is_scene_initialized_ = false;
      break;
    }
    case 'x':
    {
      ROS_INFO("[Quit] Shutting down");
      ros::shutdown();
      break;
    }
    case 'b':
    {
      ROS_INFO("[Base] Position marked");
      base_num_++;
    }
    default:
      ROS_INFO("[] Unknown command");
      break;
  }
}

bool SceneRobotControl::isSceneInitialized() 
{ 
  return is_scene_initialized_; 
}

void SceneRobotControl::waitForScene()
{
  ros::WallDuration sleep_t(1);
  while(!isSceneInitialized())
  {
    ros::spinOnce();
    ROS_WARN("[Designer] No scene set, use MoveIt plugin or publish to \"%s\"", planning_scene_topic_.c_str());
    sleep_t.sleep();
  }
}
