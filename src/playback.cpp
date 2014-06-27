#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/warehouse/planning_scene_storage.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_recorder");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  

  return 0;
}
