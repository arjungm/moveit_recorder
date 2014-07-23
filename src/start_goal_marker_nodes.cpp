#include <ros/ros.h>

#include "moveit_recorder/InteractiveRobot.h"

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "start_marker");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  InteractiveRobot robot;

  ros::spin();
    
  ros::shutdown();
  return 0;
}
