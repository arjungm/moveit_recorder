#include "moveit_recorder/AnimationRecorder.h"
#include <sstream>
#include <signal.h>

AnimationMonitor::AnimationMonitor() : m_last_msg(false), m_status(false) {}

bool AnimationMonitor::getStatus() { return m_status; }

void AnimationMonitor::statusCallback(const boost::shared_ptr<std_msgs::Bool const>& status_msg)
{
  if(!m_last_msg && status_msg->data)
  {
    m_status = true;
  }
  else if(m_last_msg && !status_msg->data)
  {
    m_status = false;
  }
  m_last_msg = status_msg->data;
}

AnimationRecorder::AnimationRecorder(std::string view_control_topic, 
                                     std::string planning_scene_topic,
                                     std::string display_traj_topic, 
                                     std::string anim_status_topic, 
                                     ros::NodeHandle& nh) 
: m_am(), m_node_handle(nh)
{
  m_view_control_pub = m_node_handle.advertise<view_controller_msgs::CameraPlacement>(view_control_topic, 1, true);
  while(m_view_control_pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("Not enough subscribers to \"%s\" topic... ", view_control_topic.c_str());
    sleep_t.sleep();
  }
  
  m_display_traj_pub = m_node_handle.advertise<moveit_msgs::DisplayTrajectory>(display_traj_topic, 1, true);
  while(m_display_traj_pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("Not enough subscribers to \"%s\" topic... ", display_traj_topic.c_str());
    sleep_t.sleep();
  }
    
  m_planning_scene_pub = m_node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1);
  while(m_planning_scene_pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("Not enough subscribers to \"%s\" topic... ", planning_scene_topic.c_str());
    sleep_t.sleep();
  }

  m_animation_status_sub = m_node_handle.subscribe(anim_status_topic, 1, &AnimationMonitor::statusCallback, &m_am);
}

AnimationRecorder::~AnimationRecorder() {}

void AnimationRecorder::record(
    const view_controller_msgs::CameraPlacement& view_msg,
    const moveit_msgs::PlanningScene& ps_msg,
    const moveit_msgs::MotionPlanRequest& mpr_msg,
    const moveit_msgs::RobotTrajectory& rt_msg,
    const std::string filepath)
{
  // set view
  ROS_INFO("Setting view");
  m_view_control_pub.publish(view_msg);

  // display scene
  ROS_INFO("Setting scene");
  m_planning_scene_pub.publish(ps_msg);
  
  // display
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = mpr_msg.start_state;
  display_trajectory.trajectory.push_back(rt_msg);
  ROS_INFO("Displaying traj");
  m_display_traj_pub.publish(display_trajectory);

  // record command
  std::stringstream ss;
  ss << "recordmydesktop -o ";
  ss << filepath;
  std::string command = ss.str();

  // fork and record
  pid_t pid;
  pid = fork();
  
  if(pid==0)
  {
    // child process records
    system(command.c_str());
  }
  else
  {
    // parent spins while the trajectory executes and kills child
    while(ros::ok() && !m_am.getStatus())
    {
      ros::spinOnce();
    }
    ROS_INFO("Animation started...");
    while(ros::ok() && m_am.getStatus())
    {
      ros::spinOnce();
    }
    ROS_INFO("Animation terminated");
    kill(0,SIGINT);
  }
}
