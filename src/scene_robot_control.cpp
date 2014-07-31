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
#include <iostream>
#include <fstream>

SceneRobotControl::SceneRobotControl(ros::NodeHandle nh, 
    const std::string& planning_scene_topic,
    const std::string& from_marker_topic,
    const std::string& from_marker_pose_topic,
    const std::string& to_marker_topic,
    const std::string& query_save_location)
: m_node_handle(nh), 
  m_scene_initialized(false), 
  m_planning_scene_topic(planning_scene_topic),
  m_from_marker_topic(from_marker_topic),
  m_from_marker_pose_topic(from_marker_pose_topic),
  m_to_marker_topic(to_marker_topic),
  m_query_num(0),
  m_query_file_location(query_save_location)
{
  // publishers
  m_planning_scene_publisher = m_node_handle.advertise<moveit_msgs::PlanningScene>(m_planning_scene_topic,1);
  m_robot_state_publisher =  m_node_handle.advertise<moveit_msgs::RobotState>(m_to_marker_topic, 1);

  // subscribers
  m_planning_scene_subscriber = m_node_handle.subscribe(m_planning_scene_topic, 1,
      &SceneRobotControl::planningSceneCallback, this);
  m_robot_state_subscriber = m_node_handle.subscribe(m_from_marker_topic, 1,
      &SceneRobotControl::markerRobotStateCallback, this);
  m_robot_pose_subscriber = m_node_handle.subscribe(m_from_marker_pose_topic, 1,
      &SceneRobotControl::markerRobotPoseCallback, this);

  // collision checking
  m_robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description")); // TODO is param?
  m_robot_model = m_robot_model_loader->getModel();
  m_robot_state.reset(new robot_state::RobotState(m_robot_model));
  m_planning_scene = boost::make_shared<planning_scene::PlanningScene>(m_robot_model);
}

SceneRobotControl::~SceneRobotControl()
{
  m_planning_scene.reset();
  m_robot_state.reset();
  m_robot_model.reset();
}

void SceneRobotControl::markerRobotPoseCallback(const boost::shared_ptr<geometry_msgs::Pose const>& msg)
{
  m_current_pose = *msg;
}

void SceneRobotControl::planningSceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
{
  if(!m_scene_initialized)
  {
    m_planning_scene->usePlanningSceneMsg(*msg);
    m_current_scene = *msg;
    m_current_state = msg->robot_state;
    m_scene_initialized = true;
    m_planning_scene_publisher.publish(m_current_scene); // to rviz & move group
    m_robot_state_publisher.publish(m_current_state); // to interactive robot
    m_query_num = 0;
  }
}

void SceneRobotControl::markerRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg)
{
  m_current_state = *msg;
  m_current_scene.robot_state = *msg;
  robot_state::robotStateMsgToRobotState(*msg, *m_robot_state);
  m_planning_scene_publisher.publish(m_current_scene); // to rviz & move group
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

void SceneRobotControl::getControlMessage(int dir)
{
  switch(dir)
  {
    case 'o':
    {
      ROS_INFO("[Output]");
      for(int i=0; i<m_current_state.joint_state.name.size();++i)
        ROS_INFO("[%35s] %6.2f", m_current_state.joint_state.name[i].c_str(), 
                                m_current_state.joint_state.position[i]);

      double roll, pitch, yaw;
      getRPY(m_current_state.multi_dof_joint_state.transforms[0], roll, pitch, yaw);
      ROS_INFO("x:%4.2f y:%4.2f z:%4.2f r:%4.2f p:%4.2f y:%4.2f",
          m_current_state.multi_dof_joint_state.transforms[0].translation.x,
          m_current_state.multi_dof_joint_state.transforms[0].translation.y,
          m_current_state.multi_dof_joint_state.transforms[0].translation.z,
          roll,
          pitch,
          yaw);

      getRPY(m_current_pose, roll, pitch, yaw);
      ROS_INFO("x:%4.2f y:%4.2f z:%4.2f r:%4.2f p:%4.2f y:%4.2f",
                m_current_pose.position.x,
                m_current_pose.position.y,
                m_current_pose.position.z,
                roll,
                pitch,
                yaw);
      break;
    }
    case 's':
    {
      ROS_INFO("[Save] %d joint positions and Pose saved so far", (int)(2*(++m_query_num)));

      std::stringstream start_prefix_ss;
      start_prefix_ss << m_current_scene.name;
      start_prefix_ss << ".start.";
      start_prefix_ss << m_query_num;
      m_query_position_names.push_back( start_prefix_ss.str() );
      
      m_query_positions.push_back(m_current_state);
      
      std::stringstream goal_prefix_ss;
      goal_prefix_ss << m_current_scene.name;
      goal_prefix_ss << ".goal.";
      goal_prefix_ss << m_query_num;
      m_query_6dofpose_names.push_back( goal_prefix_ss.str() );

      m_query_6dofposes.push_back(m_current_pose);

      break;
    }
    case 'c':
    {
      ROS_INFO("[Collision] Checking for collisions on the whole robot");
      collision_detection::CollisionRequest req; req.contacts=true;
      collision_detection::CollisionResult res;
      m_planning_scene->checkCollision(req, res, *m_robot_state, m_planning_scene->getAllowedCollisionMatrix());
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
      boost::filesystem::path query_filepath( m_query_file_location );
      query_filepath = query_filepath / m_current_scene.name;
      std::string query_file = query_filepath.string() + ".queries";
      ROS_INFO("[Write] Writing queries to file: %s", query_file.c_str());
      
      // write out to file the query locations (this saves the joint positions and poses for planning)
      writePositionsToFile( query_file, m_current_scene.name, m_query_position_names, m_query_positions);
      writePosesToFile(     query_file, m_current_scene.name, m_query_6dofpose_names, m_query_6dofposes);
      break;
    }
    case 'r':
    {
      ROS_INFO("[Reset] Scene is reset");
      m_scene_initialized = false;
      break;
    }
    case 'x':
    {
      ROS_INFO("[Quit] Shutting down");
      ros::shutdown();
      break;
    }
    default:
      ROS_INFO("[] Unknown command");
      break;
  }
}

bool SceneRobotControl::isSceneInitialized() 
{ 
  return m_scene_initialized; 
}

void SceneRobotControl::waitForScene()
{
  ros::WallDuration sleep_t(1);
  while(!isSceneInitialized())
  {
    ros::spinOnce();
    ROS_WARN("[Designer] No scene set, use MoveIt plugin or publish to \"%s\"", m_planning_scene_topic.c_str());
    sleep_t.sleep();
  }
}
