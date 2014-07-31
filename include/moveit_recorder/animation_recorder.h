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

#include <moveit_recorder/AnimationRequest.h>

#ifndef ANIM_RECORDER_H
#define ANIM_RECORDER_H

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
                      const std::string& anim_response_topic,
                      ros::NodeHandle& nh);
    ~AnimationRecorder();
    void waitOnPublishersToTopic(const ros::Subscriber& sub, const std::string& topic);
    void waitOnSubscribersToTopic(const ros::Publisher& pub, const std::string& topic);
    void record(const boost::shared_ptr<moveit_recorder::AnimationRequest>& req);
    bool getMonitorStatus();
    void forkedRecord();
    bool getRecordingReadyStatus();
    void setRecordingReadyStatus(bool status);
  private:
    AnimationMonitor am_;
    ros::Publisher view_control_pub_;
    ros::Publisher display_traj_pub_;
    ros::Publisher planning_scene_pub_;
    ros::Subscriber animation_status_sub_;
    ros::Publisher animation_response_pub_;
    ros::NodeHandle node_handle_;

    char* recorder_argv_[4];
    bool recording_ready_;
};

#endif
