#include <ros/ros.h>
#include <rosbag/bag.h>
#include <termios.h>

#include <view_controller_msgs/CameraPlacement.h>

static view_controller_msgs::CameraPlacement last_recorded_msg;

void recordViewpointCallback(const boost::shared_ptr<view_controller_msgs::CameraPlacement const>& msg)
{
  // copy
  last_recorded_msg = *msg;
  // write to bagfile
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "viewpoint_recorder");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20);
  
  ros::Subscriber sub = node_handle.subscribe("/rviz/current_camera_placement",1,recordViewpointCallback);
  while(sub.getNumPublishers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("Waiting on publishers to topic: %s", "/rviz/current_camera_placement");
    sleep_t.sleep();
  }
  
  rosbag::Bag bag("/tmp/test.bag", rosbag::bagmode::Write);
  bag.close();

  while(ros::ok())
  {
    ros::spinOnce();
    usleep(1000);

    //don't use non blocking cin...
    int c = getch();
    if(c=='s')
    {
      ROS_INFO("Writing to bag...");
      rosbag::Bag bag("/tmp/test.bag", rosbag::bagmode::Append);
      bag.write("viewpoints", ros::Time::now(), last_recorded_msg);
      bag.close();
      ROS_INFO("Saved to bag!");
    }
  }

  return 0;
}
