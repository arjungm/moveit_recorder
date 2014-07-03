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
    bool m_last_msg;
    bool m_status;
};

class AnimationRecorder
{
  public:
    AnimationRecorder(std::string view_control_topic, 
                      std::string planning_scene_topic,
                      std::string display_traj_topic,
                      std::string anim_status_topic,
                      ros::NodeHandle& nh);
    ~AnimationRecorder();
    void record(const boost::shared_ptr<moveit_recorder::AnimationRequest>& req);
  private:
    AnimationMonitor m_am;
    ros::Publisher m_view_control_pub;
    ros::Publisher m_display_traj_pub;
    ros::Publisher m_planning_scene_pub;
    ros::Subscriber m_animation_status_sub;
    ros::NodeHandle m_node_handle;
};

#endif
