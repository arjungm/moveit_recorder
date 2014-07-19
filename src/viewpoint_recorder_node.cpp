#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/program_options.hpp>
#include <view_controller_msgs/CameraPlacement.h>
#include <moveit_recorder/cli_controller.h>

static view_controller_msgs::CameraPlacement last_recorded_msg;

void recordViewpointCallback(const boost::shared_ptr<view_controller_msgs::CameraPlacement const>& msg)
{
  last_recorded_msg = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "viewpoint_recorder");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("save_dir", boost::program_options::value<std::string>(), "Directory to store the recorded bagfile of viewpoints");

  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }
  try
  {
    std::string save_dir = vm.count("save_dir") ? vm["save_dir"].as<std::string>() : "/tmp/views.bag";

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

      // use non blocking keygrab
      int c = recorder_utils::getch();
      if(c=='s')
      {
        ROS_INFO("Writing to bag...");
        rosbag::Bag bag(save_dir, rosbag::bagmode::Append);
        bag.write("viewpoints", ros::Time::now(), last_recorded_msg);
        bag.close();
        ROS_INFO("Saved to bag!");
      }
    }
  }
  catch(...)
  {
    //TODO catch possible exceptions from file io and directory creation
  }
  return 0;
}
