#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/warehouse/planning_scene_storage.h>

#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_recorder");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20); // to let RVIZ come up

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("host", boost::program_options::value<std::string>(), "Host for the MongoDB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.");

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
    //connect to the DB
    std::string host = vm.count("host") ? vm["host"].as<std::string>() : "";
    size_t port = vm.count("port") ? vm["port"].as<std::size_t>() : 0;
    moveit_warehouse::PlanningSceneStorage pss(host, port);

    ROS_INFO("Connected to Warehouse DB at host (%s) and port (%d)", host.c_str(), (int)port);
    
    //ask the warehouse for the scene
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    std::vector<std::string>::iterator scene = ps_names.begin();
    for(; scene!=ps_names.end(); ++scene)
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, *scene);
      
      // visualize the scene
      // apparently with metadata it extends it
      ps_pub.publish(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
    }

    //ask the warehouse for a trajectory

    //visualize the trajectory


    //std::vector<std::string> files = boost::program_options::collect_unrecognized(po.options, boost::program_options::include_positional);
  }
  catch(mongo_ros::DbConnectException &ex)
  {
    ROS_ERROR_STREAM("Unable to connect to warehouse. If you just created the database, it could take a while for initial setup. Please try to run the benchmark again."
        << std::endl << ex.what());
  }

  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Successfully performed trajectory playback");
  ros::shutdown();
  return 0;
}
