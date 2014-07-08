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
    ROS_INFO("Monitor: Animation started");
  }
  else if(m_last_msg && !status_msg->data)
  {
    m_status = false;
    ROS_INFO("Monitor: Animation terminated");
  }
  m_last_msg = status_msg->data;
}

AnimationRecorder::AnimationRecorder(std::string view_control_topic, 
                                     std::string planning_scene_topic,
                                     std::string display_traj_topic, 
                                     std::string anim_status_topic,
                                     std::string anim_response_topic, 
                                     ros::NodeHandle& nh) 
: m_am(), m_node_handle(nh), m_recording_ready(false)
{
  // load all pubs and subs
  m_animation_status_sub = m_node_handle.subscribe(anim_status_topic, 1, &AnimationMonitor::statusCallback, &m_am);

  m_view_control_pub = m_node_handle.advertise<view_controller_msgs::CameraPlacement>(view_control_topic, 1, true);
  m_display_traj_pub = m_node_handle.advertise<moveit_msgs::DisplayTrajectory>(display_traj_topic, 1, true);
  m_planning_scene_pub = m_node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1);
  m_animation_response_pub = m_node_handle.advertise<std_msgs::Bool>(anim_response_topic, 1);

  // wait til all topics are hooked up
  waitOnSubscribersToTopic(m_planning_scene_pub, planning_scene_topic);
  waitOnSubscribersToTopic(m_display_traj_pub,display_traj_topic);
  waitOnSubscribersToTopic(m_view_control_pub, view_control_topic);
  waitOnSubscribersToTopic(m_animation_response_pub, anim_response_topic);
  waitOnPublishersToTopic(m_animation_status_sub, anim_status_topic);
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

void AnimationRecorder::record(const boost::shared_ptr<moveit_recorder::AnimationRequest>& req)
{
  // set view
  ROS_INFO("Setting view");
  m_view_control_pub.publish(req->camera_placement);

  // display scene
  ROS_INFO("Setting scene");
  m_planning_scene_pub.publish(req->planning_scene);
  
  // display
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = req->motion_plan_request.start_state;
  display_trajectory.trajectory.push_back(req->robot_trajectory);
  ROS_INFO("Displaying traj");

  // record command
  // char* m_recorder_argv[4];
  m_recorder_argv[0] = "recordmydesktop";
  m_recorder_argv[1] = "-o";
  m_recorder_argv[2] = const_cast<char*>(req->filepath.data.c_str());
  m_recorder_argv[3] = NULL;

  m_recording_ready = true;
  
  m_display_traj_pub.publish(display_trajectory);
}

void AnimationRecorder::forkedRecord()
{
  // fork and record
  pid_t pid;
  pid = fork();

  //every process in its own process group
  if(pid==0)
  {
    // child process records
    execvp(m_recorder_argv[0], m_recorder_argv);
    exit(0);
  }
  else
  {
    std_msgs::Bool response_msg;
    response_msg.data = false;

    // parent spins while the trajectory executes and kills child
    while(ros::ok() && !getMonitorStatus())
    {
      ros::spinOnce();
      usleep(1000);
    }
    while(ros::ok() && getMonitorStatus())
    {
      ros::spinOnce();
      m_animation_response_pub.publish(response_msg);
    }
    kill(pid,SIGINT);
    usleep(1000);
    
    response_msg.data=true;
    m_animation_response_pub.publish(response_msg);
  }
}

bool AnimationRecorder::getMonitorStatus()
{
    return m_am.getStatus();
}

bool AnimationRecorder::getRecordingReadyStatus()
{
  return m_recording_ready;
}

void AnimationRecorder::setRecordingReadyStatus(bool status)
{
  m_recording_ready = status;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorder");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  AnimationRecorder recorder( "/rviz/camera_placement",
                              "planning_scene",
                              "/move_group/display_planned_path",
                              "animation_status",
                              "animation_response",
                              node_handle);

  ros::Subscriber ar_sub = node_handle.subscribe("animation_request", 1, &AnimationRecorder::record, &recorder);
  
  while(ros::ok())
  {
    ros::spinOnce();
    if( recorder.getRecordingReadyStatus() )
    {
      recorder.forkedRecord();
      recorder.setRecordingReadyStatus(false);
    }
    usleep(1000);
  }
  ROS_ERROR("Animator Node Shutdown!");
  ros::shutdown();
  return 0;
}
