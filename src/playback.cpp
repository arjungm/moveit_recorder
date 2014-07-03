#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/warehouse/planning_scene_storage.h>

#include <boost/program_options.hpp>


#include <view_controller_msgs/CameraPlacement.h>

#include "moveit_recorder/TrajectoryRetimer.h"
#include "moveit_recorder/AnimationRecorder.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "playback");
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

    AnimationRecorder recorder( "/rviz/camera_placement",
                                "planning_scene",
                                "/move_group/display_planned_path",
                                "animation_status",
                                node_handle);


    //ask the warehouse for the scene
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    
    ROS_INFO("%d available scenes to display", (int)ps_names.size());

    std::vector<std::string>::iterator scene_it = ps_names.begin();
    for(; scene_it!=ps_names.end(); ++scene_it)
    {
      ROS_INFO("Retrieving scene %s", scene_it->c_str());
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, *scene_it);
      moveit_msgs::PlanningScene ps_msg = static_cast<const moveit_msgs::PlanningScene&>(*pswm);

      // get query
      std::vector<std::string> pq_names;
      pss.getPlanningQueriesNames( pq_names, *scene_it);

      ROS_INFO("%d available queries to display", (int)pq_names.size());
      std::string first_query = pq_names.at(0);
      moveit_warehouse::MotionPlanRequestWithMetadata mprwm;
      pss.getPlanningQuery(mprwm, *scene_it, first_query);
      moveit_msgs::MotionPlanRequest mpr_msg = static_cast<const moveit_msgs::MotionPlanRequest&>(*mprwm);

      //get trajectory list
      std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
      pss.getPlanningResults(planning_results, *scene_it, first_query);

      ROS_INFO("Loaded %d trajectories for query %s", (int)planning_results.size(), first_query.c_str());

      // animate the first trajectory
      size_t ind = 0;
      
      moveit_msgs::RobotTrajectory rt_msg;
      rt_msg = static_cast<const moveit_msgs::RobotTrajectory&>(*(planning_results[ind]));

      // retime it
      TrajectoryRetimer retimer( "robot_description", mpr_msg.group_name );
      retimer.configure(ps_msg, mpr_msg);
      bool result = retimer.retime(rt_msg);
      ROS_INFO("Retiming success? %s", result? "yes" : "no" );
      
      // control the camera
      view_controller_msgs::CameraPlacement control_cam;
      control_cam.target_frame = "base_link";
      control_cam.interpolation_mode = view_controller_msgs::CameraPlacement::SPHERICAL;
      control_cam.time_from_start = ros::Duration(0.5);
      
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = "base_link";

      geometry_msgs::PointStamped eye;
      eye.header = header;
      eye.point.x = 2.5;
      eye.point.y = -1;
      eye.point.z = 2;
      geometry_msgs::PointStamped focus;
      focus.header = header;
      focus.point.x = -0.21941;
      focus.point.y = 0.27017;
      focus.point.z = 0.52922;
      geometry_msgs::Vector3Stamped up;
      up.header = header;
      up.vector.x = 0;
      up.vector.y = 0;
      up.vector.z = 1;

      control_cam.eye = eye;
      control_cam.focus = focus;
      control_cam.up = up;
      control_cam.mouse_interaction_mode = view_controller_msgs::CameraPlacement::NO_CHANGE;
      control_cam.interaction_disabled = true;
      control_cam.allow_free_yaw_axis = false;

      // record
      if(1)
      {
        recorder.record(control_cam, ps_msg, mpr_msg, rt_msg, "/tmp/video.ogv");
      }
    }
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
