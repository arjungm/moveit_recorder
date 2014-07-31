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

#include "moveit_recorder/animation_recorder.h"
#include <sstream>
#include <signal.h>
#include <sys/wait.h>


AnimationMonitor::AnimationMonitor() : last_msg_(false), status_(false) {}

bool AnimationMonitor::getStatus() { return status_; }

void AnimationMonitor::statusCallback(const boost::shared_ptr<std_msgs::Bool const>& status_msg)
{
  if(!last_msg_ && status_msg->data)
  {
    status_ = true;
    ROS_INFO("Monitor: Animation started");
  }
  else if(last_msg_ && !status_msg->data)
  {
    status_ = false;
    ROS_INFO("Monitor: Animation terminated");
  }
  last_msg_ = status_msg->data;
}

AnimationRecorder::AnimationRecorder(const std::string& view_control_topic, 
                                     const std::string& planning_scene_topic,
                                     const std::string& display_traj_topic, 
                                     const std::string& anistatus__topic,
                                     ros::NodeHandle& nh) 
: am_(), node_handle_(nh), recording_ready_(false)
{
  // load all pubs and subs
  animation_status_sub_ = node_handle_.subscribe(anistatus__topic, 1, &AnimationMonitor::statusCallback, &am_);

  view_control_pub_ = node_handle_.advertise<view_controller_msgs::CameraPlacement>(view_control_topic, 1, true);
  display_traj_pub_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(display_traj_topic, 1, true);
  planning_scene_pub_ = node_handle_.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1);

  // wait til all topics are hooked up
  waitOnSubscribersToTopic(planning_scene_pub_, planning_scene_topic);
  waitOnSubscribersToTopic(display_traj_pub_,display_traj_topic);
  waitOnSubscribersToTopic(view_control_pub_, view_control_topic);
  waitOnPublishersToTopic(animation_status_sub_, anistatus__topic);
}

AnimationRecorder::~AnimationRecorder() {}

void AnimationRecorder::waitOnSubscribersToTopic(const ros::Publisher& pub, const std::string& topic)
{
  while(pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("[Recorder] Not enough subscribers to \"%s\" topic... ", topic.c_str());
    sleep_t.sleep();
  }
}

void AnimationRecorder::waitOnPublishersToTopic(const ros::Subscriber& sub, const std::string& topic)
{
  while(sub.getNumPublishers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("[Recorder] Not enough publishers to \"%s\" topic... ", topic.c_str());
    sleep_t.sleep();
  }
}

void AnimationRecorder::record(const AnimationRequest& req)
{
  // set view
  ROS_INFO("Setting view");
  view_control_pub_.publish(req.camera_placement);

  // display scene
  ROS_INFO("Setting scene");
  planning_scene_pub_.publish(req.planning_scene);
  
  // display
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = req.motion_plan_request.start_state;
  display_trajectory.trajectory.push_back(req.robot_trajectory);
  ROS_INFO("Displaying traj");

  // record command
  // char* recorder_argv_[4];
  recorder_argv_[0] = "recordmydesktop";
  recorder_argv_[1] = "-o";
  recorder_argv_[2] = const_cast<char*>(req.filepath.c_str());
  recorder_argv_[3] = NULL;

  recording_ready_ = true;
  
  display_traj_pub_.publish(display_trajectory);
}

void AnimationRecorder::startCapture()
{
  // fork and record
  pid_t pid;
  pid = fork();

  //every process in its own process group
  if(pid==0)
  {
    // child process records
    execvp(recorder_argv_[0], recorder_argv_);
    exit(0);
  }
  else
  {
    // parent spins while the trajectory executes and kills child
    while(ros::ok() && !getMonitorStatus())
    {
      ros::spinOnce();
      usleep(1000);
    }
    while(ros::ok() && getMonitorStatus())
    {
      ros::spinOnce();
      usleep(1000);
    }
    kill(pid,SIGINT);
    usleep(1000);
    
    // monitor child until dead
    int status;
    pid_t endpid = waitpid(pid, &status, WNOHANG|WUNTRACED);
    while(endpid!=pid)
    {
      endpid = waitpid(pid, &status, WNOHANG|WUNTRACED);
      if (endpid == -1) {
        // error calling waitpid
        perror("waitpid error");
        exit(EXIT_FAILURE);
      }
      else if (endpid == 0) {        
        // child still running
        usleep(1000);
      }
      else if (endpid == pid) {
        // child ended
        if (WIFEXITED(status))
          ROS_INFO("Child ended normally.");
        else if (WIFSIGNALED(status))
          ROS_INFO("Child ended because on an uncaught signal");
        else if (WIFSTOPPED(status))
          ROS_INFO("Child process has stopped.");
      }
    }
  }
}

bool AnimationRecorder::getMonitorStatus()
{
    return am_.getStatus();
}

bool AnimationRecorder::getRecordingReadyStatus()
{
  return recording_ready_;
}

void AnimationRecorder::setRecordingReadyStatus(bool status)
{
  recording_ready_ = status;
}
