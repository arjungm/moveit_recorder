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

/* Author: Arjun Menon & Acorn Pooley */

#include "moveit_recorder/interactive_robot.h"
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_datatypes.h>

// minimum delay between calls to callback function
const ros::Duration InteractiveRobot::min_delay_(0.01);

InteractiveRobot::InteractiveRobot(
    const std::string& robot_description,
    const std::string& planning_scene_topic,
    const std::string& from_scene_topic,
    const std::string& to_scene_topic,
    const std::string& to_scene_pose_topic,
    const std::string& display_robot_topic,
    const std::string& marker_topic,
    const std::string& imarker_topic,
    const std::string& arm_name) :
  // this node handle is used to create the publishers
  nh_(),
  // create publishers for markers and robot state
  robot_state_publisher_(nh_.advertise<moveit_msgs::DisplayRobotState>(display_robot_topic,1)),
  // create an interactive marker server for displaying interactive markers
  imarker_robot_(0),
  imarker_base_(0),
  imarker_other_(0),
  // load the robot description
  rm_loader_(robot_description),
  group_(0),
  user_data_(0),
  user_callback_(0),
  robot_state_initialized_(false),
  base_changed_(false),
  arm_name_(arm_name),
  other_arm_name_(arm_name_.at(0)=='r'?"left_arm":"right_arm")
{
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer(imarker_topic)),
  // get the RobotModel loaded from urdf and srdf files
  robot_model_ = rm_loader_.getModel();
  if (!robot_model_) {
    ROS_ERROR("Could not load robot description");
    throw RobotLoadException();
  }
  ROS_INFO("Interactivity Started");
  // create a RobotState to keep track of the current robot pose
  marker_robot_state_publisher_ = nh_.advertise<moveit_msgs::RobotState>(to_scene_topic,1);
  marker_robot_pose_publisher_ = nh_.advertise<geometry_msgs::Pose>(to_scene_pose_topic,1);
  robot_state_subscriber_ = nh_.subscribe(from_scene_topic,1,&InteractiveRobot::updateRobotStateCallback, this);
  // load from message
  ros::WallDuration wait_t(1);
  while( robot_state_subscriber_.getNumPublishers()<1 )
  {
    ROS_WARN("[Interactivity] Current state publisher is not loaded, for topic \"%s\"", from_scene_topic.c_str());
    wait_t.sleep();
  }
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  while( !robot_state_initialized_ )
  {
    ros::spinOnce();
    ROS_WARN("[Interactivity] Current scene's robot state is not loaded, spinning...");
    wait_t.sleep();
  }

  if (!robot_state_) {
    ROS_ERROR("Could not get RobotState from Model");
    throw RobotLoadException();
  }
  base_joint_ = robot_state_->getJointModel("world_joint");
  
  ROS_INFO("Loading groups in for \"%s\" and \"%s\"", arm_name_.c_str(), other_arm_name_.c_str());

  // Prepare to move the "right_arm" group
  group_ = robot_model_->getJointModelGroup(arm_name_);
  end_link_ = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link_);
  desired_base_link_pose_ = robot_state_->getGlobalLinkTransform("base_link");

  // Other
  other_group_ = robot_model_->getJointModelGroup(other_arm_name_);
  other_end_link_ = other_group_->getLinkModelNames().back();
  other_desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(other_end_link_);
  
  // Create a marker to control the "right_arm" group
  imarker_robot_ = new IMarker(*interactive_marker_server_,
                               arm_name_,
                               desired_group_end_link_pose_,
                               "/base_footprint",
                               boost::bind(movedRobotMarkerCallback,this,_1),
                               IMarker::BOTH);
  imarker_other_ = new IMarker(*interactive_marker_server_,
                               other_arm_name_,
                               other_desired_group_end_link_pose_,
                               "/base_footprint",
                               boost::bind(movedOtherMarkerCallback,this,_1),
                               IMarker::BOTH);
  imarker_base_ = new IMarker(*interactive_marker_server_, 
                              "base", 
                              desired_base_link_pose_, 
                              "/base_footprint",
                              boost::bind(movedRobotBaseMarkerCallback, this, _1), 
                              IMarker::PLANAR);
  

  // start publishing timer.
  init_time_ = ros::Time::now();
  last_callback_time_ = init_time_;
  average_callback_duration_ = min_delay_;
  schedule_request_count_ = 0;
  publish_timer_ = nh_.createTimer(average_callback_duration_, &InteractiveRobot::updateCallback, this, true);

  // begin publishing robot state
  scheduleUpdate();
}

InteractiveRobot::~InteractiveRobot()
{
  delete imarker_robot_;
  delete imarker_other_;
  delete imarker_base_;
  interactive_marker_server_.reset();
}

void InteractiveRobot::resetMarkers()
{
  delete imarker_robot_;
  delete imarker_other_;
  delete imarker_base_;
  
  imarker_robot_ = new IMarker(*interactive_marker_server_,
                               arm_name_,
                               desired_group_end_link_pose_,
                               "/base_footprint",
                               boost::bind(movedRobotMarkerCallback,this,_1),
                               IMarker::BOTH);
  imarker_other_ = new IMarker(*interactive_marker_server_,
                               other_arm_name_,
                               other_desired_group_end_link_pose_,
                               "/base_footprint",
                               boost::bind(movedOtherMarkerCallback,this,_1),
                               IMarker::BOTH);
  imarker_base_ = new IMarker(*interactive_marker_server_, 
                              "base", 
                              desired_base_link_pose_, 
                              "/base_footprint",
                              boost::bind(movedRobotBaseMarkerCallback, this, _1), 
                              IMarker::PLANAR);
}

void InteractiveRobot::updateRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg)
{
  robot_state_initialized_ = true;
  bool result = robot_state::robotStateMsgToRobotState(*msg, *robot_state_);
  if(!result)
    ROS_WARN("Failed to parse Robot State Message into existing robot state");
  scheduleUpdate();
}

// callback called when marker moves.  Moves right hand to new marker pose.
void InteractiveRobot::movedRobotMarkerCallback(
    InteractiveRobot *robot,
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(feedback->pose, pose);
  robot->setGroupPose(pose);
}

// callback called when marker moves.  Moves right hand to new marker pose.
void InteractiveRobot::movedOtherMarkerCallback(
    InteractiveRobot *robot,
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(feedback->pose, pose);
  robot->setOtherGroupPose(pose);
}

void InteractiveRobot::movedRobotBaseMarkerCallback(InteractiveRobot *robot, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  double x, y, theta;
  x = feedback->pose.position.x;
  y = feedback->pose.position.y;
  theta = tf::getYaw(feedback->pose.orientation);

  robot->setBasePose(x,y,theta);
}

// set the callback timer to fire if needed.
// Return true if callback should happen immediately
bool InteractiveRobot::setCallbackTimer(bool new_update_request)
{
  publish_timer_.stop();

  const ros::Time now = ros::Time::now();
  const ros::Duration desired_delay = std::max(min_delay_, average_callback_duration_ * 1.2);
  ros::Duration sec_since_last_callback = now - last_callback_time_;
  ros::Duration sec_til_next_callback = desired_delay - sec_since_last_callback;

  if (schedule_request_count_)
  {
    // need a callback desired_delay seconds after previous callback
    schedule_request_count_ += new_update_request ? 1 : 0;
    if (sec_til_next_callback <= ros::Duration(0.0001))
    {
      // just run the callback now
      return true;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (new_update_request)
  {
    if (sec_til_next_callback < min_delay_) {
      // been a while.  Use min_delay_.
      // Set last_callback_time_ to prevent firing too early
      sec_til_next_callback = min_delay_;
      sec_since_last_callback = desired_delay - sec_til_next_callback;
      last_callback_time_ = now - sec_since_last_callback;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (!init_time_.isZero())
  {
    // for the first few seconds after startup call the callback periodically
    // to ensure rviz gets the initial state.
    // Without this rviz does not show some state until markers are moved.
    if ((now - init_time_).sec >= 8)
    {
      init_time_ = ros::Time(0,0);
      return false;
    }
    else
    {
      publish_timer_.setPeriod(std::max(ros::Duration(1.0), average_callback_duration_*2));
      publish_timer_.start();
      return false;
    }
  }
  else
  {
    // nothing to do.  No callback needed.
    return false;
  }
}

// Indicate that the world or the robot has changed and
// the new state needs to be updated and published to rviz
void InteractiveRobot::scheduleUpdate()
{
  // schedule an update callback for the future.
  // If the callback should run now, call it.
  if (setCallbackTimer(true))
    updateCallback(ros::TimerEvent());
}


/* callback called when it is time to publish */
void InteractiveRobot::updateCallback(const ros::TimerEvent& e)
{
  ros::Time tbegin = ros::Time::now();
  publish_timer_.stop();

  // do the actual calculations and publishing
  updateAll();

  // measure time spent in callback for rate limiting
  ros::Time tend = ros::Time::now();
  average_callback_duration_ = (average_callback_duration_ + (tend - tbegin)) * 0.5;
  last_callback_time_ = tend;
  schedule_request_count_ = 0;

  // schedule another callback if needed
  setCallbackTimer(false);
}

/* Calculate new positions and publish results to rviz */
void InteractiveRobot::updateAll()
{
  //TODO get const *

  if (robot_state_->setFromIK(robot_state_->getJointModelGroup(arm_name_), desired_group_end_link_pose_, 10, 0.1))
    publishRobotState();
  if(robot_state_->setFromIK(robot_state_->getJointModelGroup(other_arm_name_), other_desired_group_end_link_pose_, 10, 0.1))
    publishRobotState();
  if(base_changed_)
  {
    publishRobotState();
    base_changed_ = false;
  }
}

// change which group is being manipulated
void InteractiveRobot::setGroup(const std::string& name)
{
  robot_state::JointModelGroup* group = robot_model_->getJointModelGroup(name);
  if (!group)
  {
    ROS_ERROR_STREAM("No joint group named " << name);
    if (!group_)
      throw RobotLoadException();
  }
  group_ = group;
  end_link_ = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link_);
  if (imarker_robot_)
  {
    imarker_robot_->move(desired_group_end_link_pose_);
  }
}

// return current group name
const std::string& InteractiveRobot::getGroupName() const
{
  return group_->getName();
}

/* remember new desired robot pose and schedule an update */
bool InteractiveRobot::setGroupPose(const Eigen::Affine3d& pose)
{
  desired_group_end_link_pose_ = pose;
  pose_of_ee_in_base_ = desired_base_link_pose_.inverse() * pose;
  scheduleUpdate();
}

bool InteractiveRobot::setOtherGroupPose(const Eigen::Affine3d& pose)
{
  other_desired_group_end_link_pose_ = pose;
  other_pose_of_ee_in_base_  = desired_base_link_pose_.inverse() * pose;
  scheduleUpdate();
}

bool InteractiveRobot::setBasePose(double x, double y, double theta)
{
  std::vector<double> positions;
  positions.push_back(x);
  positions.push_back(y);
  positions.push_back(theta);
  robot_state_->setJointPositions(base_joint_, positions);

  desired_base_link_pose_ = robot_state_->getGlobalLinkTransform("base_link");
  
  desired_group_end_link_pose_ = pose_of_ee_in_base_ * desired_base_link_pose_;
  other_desired_group_end_link_pose_ = other_pose_of_ee_in_base_ * desired_base_link_pose_;

  // resetMarkers();

  imarker_robot_->move(desired_group_end_link_pose_);
  imarker_other_->move(other_desired_group_end_link_pose_);

  base_changed_=true;
  scheduleUpdate();
}

/* publish robot pose to rviz */
void InteractiveRobot::publishRobotState()
{
  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(*robot_state_, msg.state);
  robot_state_publisher_.publish(msg);
  
  moveit_msgs::RobotState rs_msg;
  robot_state::robotStateToRobotStateMsg(*robot_state_, rs_msg);
  marker_robot_state_publisher_.publish(rs_msg);

  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d pose = robot_state_->getGlobalLinkTransform(end_link_);
  tf::poseEigenToMsg(pose, pose_msg);
  marker_robot_pose_publisher_.publish(pose_msg);
}
