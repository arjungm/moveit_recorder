#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

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


    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //ask the warehouse for the scene
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    
    ROS_INFO("%d available scenes to display", (int)ps_names.size());

    std::vector<std::string>::iterator scene_it = ps_names.begin();
    for(; scene_it!=ps_names.end(); ++scene_it)
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, *scene_it);

      // visualize the scene_it
      // apparently with metadata it extends it
      moveit_msgs::PlanningScene ps_msg = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
      ps_pub.publish(ps_msg);

      //get query list
      std::vector<std::string> pq_names;
      pss.getPlanningQueriesNames( pq_names, *scene_it);

      ROS_INFO("%d available queries to display", (int)pq_names.size());
      std::string first_query = pq_names.at(0);

      //get trajectory list
      std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
      pss.getPlanningResults(planning_results, *scene_it, first_query);

      ROS_INFO("Loaded %d trajectories for query %s", (int)planning_results.size(), first_query.c_str());

      //animate the first trajectory
      size_t last = planning_results.size()-1;
      
      moveit_msgs::RobotTrajectory rt_msg;
      rt_msg = static_cast<const moveit_msgs::RobotTrajectory&>(*(planning_results[last]));
      
      //get the start point
      moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
      pss.getPlanningQuery(planning_query, *scene_it, first_query);
      moveit_msgs::MotionPlanRequest mpr = static_cast<const moveit_msgs::MotionPlanRequest&>(*planning_query);
      
      //publish
      if(1)
      {
        display_trajectory.trajectory_start = mpr.start_state;
        display_trajectory.trajectory.push_back(rt_msg);
        display_publisher.publish(display_trajectory);
        sleep(5.0);
      }
    }

    //std::vector<std::string> files = boost::program_options::collect_unrecognized(po.options, boost::program_options::include_positional);
  }
  catch(mongo_ros::DbConnectException &ex)
  {
    ROS_ERROR_STREAM("Unable to connect to warehouse. If you just created the database, it could take a while for initial setup. Please try to run the benchmark again."
        << std::endl << ex.what());
  }


  ROS_INFO("Successfully performed trajectory playback");
  ros::shutdown();
  return 0;
}
