#include "moveit_recorder/SceneRobotControl.h"

SceneRobotControl::SceneRobotControl(ros::NodeHandle nh, 
    const std::string& planning_scene_topic,
    const std::string& from_marker_topic,
    const std::string& to_marker_topic,
    const std::string& query_save_location)
: m_node_handle(nh), 
  m_scene_initialized(false), 
  m_planning_scene_topic(planning_scene_topic),
  m_from_marker_topic(from_marker_topic),
  m_to_marker_topic(to_marker_topic),
  m_query_num(0)
{
  // publishers
  m_planning_scene_publisher = m_node_handle.advertise<moveit_msgs::PlanningScene>(m_planning_scene_topic,1);
  m_robot_state_publisher =  m_node_handle.advertise<moveit_msgs::RobotState>(m_to_marker_topic, 1);

  // subscribers
  m_planning_scene_subscriber = m_node_handle.subscribe(m_planning_scene_topic, 1,
      &SceneRobotControl::planningSceneCallback,
      this);
  m_robot_state_subscriber = m_node_handle.subscribe(m_from_marker_topic, 1,
      &SceneRobotControl::markerRobotStateCallback,
      this);
}

SceneRobotControl::~SceneRobotControl(){}

void SceneRobotControl::planningSceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
{
  if(!m_scene_initialized)
  {
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
  m_planning_scene_publisher.publish(m_current_scene); // to rviz & move group
}

void SceneRobotControl::getControlMessage(int dir)
{
  switch(dir)
  {
    case 's':
    {
      ROS_INFO("[Save] %d joint positions and Pose saved so far", (int)(2*(++m_query_num)));
      break;
    }
    case 'w':
    {
      std::string query_file = m_current_scene.name + ".queries";
      ROS_INFO("[Write] Writing queries to file: %s", query_file.c_str());
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
