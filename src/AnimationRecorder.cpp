#include "moveit_recorder/AnimationRecorder.h"
#include <moveit_recorder/AnimationResponse.h>
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
                                     ros::NodeHandle& nh) 
: m_am(), m_node_handle(nh), m_record_start(false)
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

  //set status to ready to go
  m_record_start = true;

  m_display_traj_pub.publish(display_trajectory);

}

bool AnimationRecorder::getRecordStatus()
{
    return m_record_start;
}

void AnimationRecorder::setRecordStatus(bool status)
{
    m_record_start = status;
}

bool AnimationRecorder::getMonitorStatus()
{
    return m_am.getStatus();
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
                              node_handle);

  ros::Subscriber ar_sub = node_handle.subscribe("animation_request", 1, &AnimationRecorder::record, &recorder);
  ros::Publisher ar_pub = node_handle.advertise<moveit_recorder::AnimationResponse>("animation_response",1,true);
  
  while(ros::ok())
  {
    ros::spinOnce();

    //if anim request received
    if(recorder.getRecordStatus())
    {
        // record command
        char* argv[4];
        argv[0] = "recordmydesktop";
        argv[1] = "-o";
        argv[2] = "/tmp/video.ogv"; //const_cast<char*>(req->filepath.data.c_str());
        argv[3] = NULL;

        // fork and record
        pid_t pid;
        pid = fork();

        //every process in its own process group
        if(pid==0)
        {
            // child process records
            execvp(argv[0], argv);
            exit(0);
        }
        else
        {
            moveit_recorder::AnimationResponse ar;
            // parent spins while the trajectory executes and kills child
            while(ros::ok() && !recorder.getMonitorStatus())
            {
                ros::spinOnce();
                ar.ready.data = false;
                ar.complete.data = false;
                ar_pub.publish(ar);
            }
            ROS_WARN("Animation started...");
            ar.ready.data = false;
            ar.complete.data = false;
            ar_pub.publish(ar);
            while(ros::ok() && recorder.getMonitorStatus())
            {
                ros::spinOnce();
                ar_pub.publish(ar);
            }
            ROS_WARN("Animation terminated...");
            recorder.setRecordStatus(false);
            usleep(1000);
            kill(pid,SIGINT);
            ar.ready.data=true;
            ar.complete.data=true;
            ar_pub.publish(ar);
        }
    }
    usleep(1000);
  }
  ROS_ERROR("Animator Node Shutdown!");
  ros::shutdown();
  return 0;
}
