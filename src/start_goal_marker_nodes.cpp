#include <ros/ros.h>
#include <boost/program_options.hpp>
#include "moveit_recorder/InteractiveRobot.h"

inline std::string get_option(const boost::program_options::variables_map& vm, const std::string& option, const std::string& default_str)
{
  return vm.count(option) ? vm[option].as<std::string>() : default_str;
}

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "start_marker");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("robot_description", boost::program_options::value<std::string>(), "robot description param name")
    ("to_marker_topic", boost::program_options::value<std::string>(), "robot state topic FROM planning scene")
    ("from_marker_topic", boost::program_options::value<std::string>(), "robot state topic TO planning scene")
    ("from_marker_pose_topic", boost::program_options::value<std::string>(), "pose topic for robot end link pose")
    ("display_robot_topic", boost::program_options::value<std::string>(), "display robot state topic")
    ("robot_marker_topic", boost::program_options::value<std::string>(), "topic for robot visual markers")
    ("interactive_marker_topic", boost::program_options::value<std::string>(), "topic for interactive marker");

  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }
  try
  {
    std::string robot_description = get_option(vm, "robot_description", "robot_description");
    std::string to_marker_topic = get_option(vm, "to_marker_topic", "to_marker_state");
    std::string from_marker_topic = get_option(vm, "from_marker_topic", "from_marker_state");
    std::string from_marker_pose_topic = get_option(vm, "from_marker_pose_topic", "from_marker_pose");
    std::string display_robot_topic = get_option(vm, "display_robot_topic", "interactive_robot_state");
    std::string robot_marker_topic = get_option(vm, "robot_marker_topic", "interactive_robot_markers");
    std::string interactive_marker_topic = get_option(vm, "interactive_marker_topic", "interactive_robot_imarkers");

    InteractiveRobot robot( robot_description,
                            to_marker_topic,
                            from_marker_topic,
                            from_marker_pose_topic,
                            display_robot_topic,
                            robot_marker_topic,
                            interactive_marker_topic);
    ros::spin();
  }
  catch(...)
  {

  }

  ros::shutdown();
  return 0;
}
