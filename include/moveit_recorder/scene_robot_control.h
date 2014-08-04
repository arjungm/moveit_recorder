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

#ifndef SCENE_ROBOT_CONTROL_H
#define SCENE_ROBOT_CONTROL_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

inline void getRPY(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw)
{
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
}
inline void getRPY(const geometry_msgs::Transform& transform, double& roll, double& pitch, double& yaw)
{
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(transform.rotation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
}

class SceneRobotControl
{
  public:
    SceneRobotControl(ros::NodeHandle nh, 
        const std::string& planning_scene_topic = "planning_scene",
        const std::string& from_marker_topic = "from_marker_state",
        const std::string& from_marker_pose_topic = "from_marker_pose",
        const std::string& to_marker_topic = "to_marker_state",
        const std::string& query_save_location = "/tmp");
    ~SceneRobotControl();
    void planningSceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg);
    void markerRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg);
    void markerRobotPoseCallback(const boost::shared_ptr<geometry_msgs::Pose const>& msg);
    void getControlMessage(int dir);
    bool isSceneInitialized();
    void waitForScene();
  protected:

    void writePositionsToFile(const std::string& filepath, 
                              const std::string& scene_name,
                              const std::vector<std::string>& names,
                              const std::vector<moveit_msgs::RobotState>& positions);
    void writePosesToFile(const std::string& filepath,
                          const std::string& scene_name,
                          const std::vector<std::string>& names,
                          const std::vector<geometry_msgs::Pose>& poses);
    
  private:
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;
    planning_scene::PlanningScenePtr planning_scene_;

    moveit_msgs::PlanningScene current_scene_;
    moveit_msgs::RobotState current_state_;
    geometry_msgs::Pose current_pose_;

    ros::Publisher planning_scene_publisher_;
    ros::Publisher robot_state_publisher_;

    ros::Subscriber planning_scene_subscriber_;
    ros::Subscriber robot_state_subscriber_;
    ros::Subscriber robot_pose_subscriber_;

    ros::NodeHandle node_handle_;

    std::string planning_scene_topic_;
    std::string from_marker_topic_;
    std::string from_marker_pose_topic_;
    std::string to_marker_topic_;

    std::string query_file_location_;

    bool is_scene_initialized_;

    size_t query_num_;
    std::vector<std::string> query_position_names_;
    std::vector<std::string> query_6dofpose_names_;
    std::vector<moveit_msgs::RobotState> query_positions_;
    std::vector<geometry_msgs::Pose> query_6dofposes_;
};

#endif
