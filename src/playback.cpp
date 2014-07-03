#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/warehouse/planning_scene_storage.h>

#include <boost/program_options.hpp>


#include <view_controller_msgs/CameraPlacement.h>

#include "moveit_recorder/TrajectoryRetimer.h"
#include "moveit_recorder/AnimationRecorder.h"

#include <moveit_recorder/AnimationRequest.h>

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

    // request pub
    ros::Publisher animation_pub = node_handle.advertise<moveit_recorder::AnimationRequest>("animation_request",1);
    while(animation_pub.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      ROS_INFO("Not enough subscribers to \"%s\" topic... ", "animation_request");
      sleep_t.sleep();
    }


    // control the camera
    view_controller_msgs::CameraPlacement view_msg;
    view_msg.target_frame = "base_link";
    view_msg.interpolation_mode = view_controller_msgs::CameraPlacement::SPHERICAL;
    view_msg.time_from_start = ros::Duration(0.5);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "base_link";
    view_msg.eye.header = header;
    view_msg.eye.point.x = 2.5;
    view_msg.eye.point.y = -1;
    view_msg.eye.point.z = 2;
    view_msg.focus.header = header;
    view_msg.focus.point.x = -0.21941;
    view_msg.focus.point.y = 0.27017;
    view_msg.focus.point.z = 0.52922;
    view_msg.up.header = header;
    view_msg.up.vector.x = 0;
    view_msg.up.vector.y = 0;
    view_msg.up.vector.z = 1;
    view_msg.mouse_interaction_mode = view_controller_msgs::CameraPlacement::NO_CHANGE;
    view_msg.interaction_disabled = true;
    view_msg.allow_free_yaw_axis = false;

    // ask the warehouse for the scenes
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    ROS_INFO("%d available scenes to display", (int)ps_names.size());
    
    // iterate over scenes
    std::vector<std::string>::iterator scene_name = ps_names.begin();
    for(; scene_name!=ps_names.end(); ++scene_name)
    {
      ROS_INFO("Retrieving scene %s", scene_name->c_str());
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, *scene_name);
      moveit_msgs::PlanningScene ps_msg = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
      
      // ask qarehosue for the queries
      std::vector<std::string> pq_names;
      pss.getPlanningQueriesNames( pq_names, *scene_name);
      ROS_INFO("%d available queries to display", (int)pq_names.size());
      
      // iterate over the queries
      std::vector<std::string>::iterator query_name = pq_names.begin();
      for(; query_name!=pq_names.end(); ++query_name)
      {
        ROS_INFO("Retrieving query %s", query_name->c_str());
        moveit_warehouse::MotionPlanRequestWithMetadata mprwm;
        pss.getPlanningQuery(mprwm, *scene_name, *query_name);
        moveit_msgs::MotionPlanRequest mpr_msg = static_cast<const moveit_msgs::MotionPlanRequest&>(*mprwm);

        // ask warehouse for stored trajectories
        std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
        pss.getPlanningResults(planning_results, *scene_name, *query_name);
        ROS_INFO("Loaded %d trajectories", (int)planning_results.size());

        // animate each trajectory
        std::vector<moveit_warehouse::RobotTrajectoryWithMetadata>::iterator traj_w_mdata = planning_results.begin();
        for(; traj_w_mdata!=planning_results.end(); ++traj_w_mdata)
        {

          moveit_msgs::RobotTrajectory rt_msg;
          rt_msg = static_cast<const moveit_msgs::RobotTrajectory&>(**traj_w_mdata);
          // retime it
          TrajectoryRetimer retimer( "robot_description", mpr_msg.group_name );
          retimer.configure(ps_msg, mpr_msg);
          bool result = retimer.retime(rt_msg);
          ROS_INFO("Retiming success? %s", result? "yes" : "no" );

          moveit_recorder::AnimationRequest req;
          req.camera_placement = view_msg;
          req.planning_scene = ps_msg;
          req.motion_plan_request = mpr_msg;
          req.robot_trajectory = rt_msg;
          req.filepath.data = "/tmp/video.ogv";

          animation_pub.publish(req);
          
          std::cout << "HALT PLAYBACK" << std::endl;
          std::cin.get();
        }//traj
      }//query
    }//scene
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
