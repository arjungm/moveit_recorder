#include "moveit_recorder/SceneRobotControl.h"
#include <boost/filesystem.hpp>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

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
  m_query_num(0)
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
}

SceneRobotControl::~SceneRobotControl(){}

void SceneRobotControl::markerRobotPoseCallback(const boost::shared_ptr<geometry_msgs::Pose const>& msg)
{
  m_current_pose = *msg;
}

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
    case 'o':
    {
      for(int i=0; i<m_current_state.joint_state.name.size();++i)
        ROS_INFO("[%40s] %20.2f", m_current_state.joint_state.name[i].c_str(), 
                                m_current_state.joint_state.position[i]);

      tf::Quaternion orientation;
      tf::quaternionMsgToTF(m_current_pose.orientation, orientation);
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

      ROS_INFO("x:%4.2f y:%4.2f z:%4.2f r:%4.2f p:%4.2f y:%4.2f",
                m_current_pose.position.x,
                m_current_pose.position.y,
                m_current_pose.position.z,
                roll,
                pitch,
                yaw);
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

      break;
    }
    case 'w':
    {
      boost::filesystem::path query_filepath( m_query_file_location );
      query_filepath /= m_current_scene.name;
      std::string query_file = query_filepath.leaf().string() + ".queries";
      ROS_INFO("[Write] Writing queries to file: %s", query_file.c_str());
      
      // write out to file the query locations (this saves the joint positions and poses for planning)

      // overwrite the warehouse scene info (this saves the robot world position)

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
