#include <ros/ros.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit_recorder/SceneRobotControl.h>

inline std::string get_option(const boost::program_options::variables_map& vm, const std::string& option, const std::string& default_str)
{
  return vm.count(option) ? vm[option].as<std::string>() : default_str;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "designer");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("host", boost::program_options::value<std::string>(), "Host for the Mongo database")
    ("port", boost::program_options::value<std::size_t>(), "Port for the Mongo database")
    ("planning_scene_topic", boost::program_options::value<std::string>(), "Planning Scene Topic for the UI")
    ("from_marker_topic",boost::program_options::value<std::string>(), "Topic on RobotState from interactive marker")
    ("from_marker_pose_topic",boost::program_options::value<std::string>(), "Topic on Robot Pose from interactive marker")
    ("to_marker_topic",boost::program_options::value<std::string>(), "Topic on Robot State towards interactive marker")
    ("save_dir", boost::program_options::value<std::string>(), "Directory to store the recorded bagfile of viewpoints");

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
    // connect to the database
    std::string host = vm.count("host") ? vm["host"].as<std::string>() : "127.0.0.1";
    size_t port = vm.count("port") ? vm["port"].as<std::size_t>() : 33829;
    moveit_warehouse::PlanningSceneStorage pss(host, port);
    ROS_INFO("Connected to Warehouse DB at host (%s) and port (%d)", host.c_str(), (int)port);

    // planning scene connection for editing in real time
    // publish diffs to change the robot status
    std::string planning_scene_topic = get_option(vm, "planning_scene_topic", "planning_scene");
    std::string from_marker_topic = get_option(vm, "from_marker_topic", "from_marker_state");
    std::string from_marker_pose_topic = get_option(vm, "from_marker_pose_topic", "from_marker_pose");
    std::string to_marker_topic = get_option(vm, "to_marker_topic", "to_marker_state");
    std::string query_save_location = get_option(vm, "save_dir", "/tmp/");

    // initialize the scene and control parser
    SceneRobotControl brc(node_handle, 
                         planning_scene_topic, 
                         from_marker_topic,
                         from_marker_pose_topic,
                         to_marker_topic,
                         query_save_location);
    brc.waitForScene();
    
    // spin and catch results
    moveit_msgs::PlanningScene update_scene;
    while(ros::ok())
    {
      ros::spinOnce();
      usleep(1000);
      
      char key = recorder_utils::getch();
      
      // process the command
      brc.getControlMessage(key);

      // if the keypress is a reset, then a new scene needs to be loaded
      brc.waitForScene();
    }
  }
  catch(mongo_ros::DbConnectException &ex)
  {
    ROS_ERROR_STREAM("Unable to connect to warehouse. If you just created the database, it could take a while for initial setup. Please try to run the benchmark again." << std::endl << ex.what());
  }
  catch(...)
  {
    //TODO catch possible exceptions from file io and directory creation
  }
  ros::shutdown();
  return 0;
}
