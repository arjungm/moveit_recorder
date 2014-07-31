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
#include <string>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <stdio.h>
#include <sys/types.h>
#include <std_msgs/Bool.h>

#include <view_controller_msgs/CameraPlacement.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/PlanningScene.h>

#ifndef ANIM_RECORDER_H
#define ANIM_RECORDER_H

struct AnimationRequest
{
  view_controller_msgs::CameraPlacement camera_placement;
  moveit_msgs::PlanningScene planning_scene;
  moveit_msgs::MotionPlanRequest motion_plan_request;
  moveit_msgs::RobotTrajectory robot_trajectory;
  std::string filepath;
};

class AnimationMonitor
{
  public:
    AnimationMonitor();
    void statusCallback(const boost::shared_ptr<std_msgs::Bool const>& status_msg);
    bool getStatus();
  private:
    bool last_msg_;
    bool status_;
};

class AnimationRecorder
{
  public:
    AnimationRecorder(const std::string& view_control_topic, 
                      const std::string& planning_scene_topic,
                      const std::string& display_traj_topic,
                      const std::string& anistatus__topic,
                      ros::NodeHandle& nh);
    ~AnimationRecorder();
    void waitOnPublishersToTopic(const ros::Subscriber& sub, const std::string& topic);
    void waitOnSubscribersToTopic(const ros::Publisher& pub, const std::string& topic);
    void record(const AnimationRequest& req);
    bool getMonitorStatus();
    void startCapture();
    bool getRecordingReadyStatus();
    void setRecordingReadyStatus(bool status);
  private:
    AnimationMonitor am_;
    ros::Publisher view_control_pub_;
    ros::Publisher display_traj_pub_;
    ros::Publisher planning_scene_pub_;
    ros::Subscriber animation_status_sub_;
    ros::NodeHandle node_handle_;

    char* recorder_argv_[4];
    bool recording_ready_;
};

#endif
