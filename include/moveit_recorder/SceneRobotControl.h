#ifndef SCENE_ROBOT_CONTROL_H
#define SCENE_ROBOT_CONTROL_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>

class SceneRobotControl
{
  public:
    SceneRobotControl(ros::NodeHandle nh, 
        const std::string& planning_scene_topic = "planning_scene",
        const std::string& from_marker_topic = "from_marker_state",
        const std::string& to_marker_topic = "to_marker_state",
        const std::string& query_save_location = "/tmp");
    ~SceneRobotControl();
    void planningSceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg);
    void markerRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg);
    void getControlMessage(int dir);
    bool isSceneInitialized();
    void waitForScene();
  private:

    moveit_msgs::PlanningScene m_current_scene;
    moveit_msgs::RobotState m_current_state;

    ros::Publisher m_planning_scene_publisher;
    ros::Publisher m_robot_state_publisher;

    ros::Subscriber m_planning_scene_subscriber;
    ros::Subscriber m_robot_state_subscriber;

    ros::NodeHandle m_node_handle;

    std::string m_planning_scene_topic;
    std::string m_from_marker_topic;
    std::string m_to_marker_topic;

    bool m_scene_initialized;

    size_t m_query_num;
    std::vector< std::string > m_query_position_names;
    std::vector< std::string > m_query_6dofpose_names;
    std::vector< double > m_query_positions;
    std::vector< double > m_query_6dofposes;
};

#endif
