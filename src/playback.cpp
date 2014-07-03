#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <boost/program_options.hpp>

#include <stdio.h>
#include <sys/types.h>
#include <std_msgs/Bool.h>

#include <rviz/view_controller.h>
#include <rviz_animated_view_controller/rviz_animated_view_controller.h>

class AnimationMonitor
{
  public:
    AnimationMonitor() : last_msg_(false), status_(false) {}
    void statusCallback(const boost::shared_ptr<std_msgs::Bool const>& status_msg)
    {
      if(!last_msg_ && status_msg->data)
      {
        status_ = true;
      }
      else if(last_msg_ && !status_msg->data)
      {
        status_ = false;
      }
      last_msg_ = status_msg->data;
    }
    bool getStatus()
    {
      return status_;
    }
  private:
    bool last_msg_;
    bool status_;
};

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

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //ask the warehouse for the scene
    std::vector<std::string> ps_names;
    pss.getPlanningSceneNames( ps_names );
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(ps_pub.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      ROS_INFO("Not enough subscribers to \"%s\" topic... ", "planning_scene");
      sleep_t.sleep();
    }
    
    ROS_INFO("%d available scenes to display", (int)ps_names.size());

    std::vector<std::string>::iterator scene_it = ps_names.begin();
    for(; scene_it!=ps_names.end(); ++scene_it)
    {
      ROS_INFO("Retrieving scene %s", scene_it->c_str());
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, *scene_it);

      // visualize the scene
      ROS_INFO("Publishing scene...");
      moveit_msgs::PlanningScene ps_msg = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
      ps_pub.publish(ps_msg);

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
      
      // retime the trajectory
      trajectory_processing::IterativeParabolicTimeParameterization traj_retimer;
      planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
      planning_scene::PlanningScenePtr ps = psm.getPlanningScene();
      ps->setPlanningSceneMsg( ps_msg );

      robot_trajectory::RobotTrajectory rt(ps->getRobotModel(), mpr_msg.group_name);
      moveit::core::RobotState rs(ps->getRobotModel());

      rs.setVariableValues(mpr_msg.start_state.joint_state); //ref state
      rt.setRobotTrajectoryMsg(rs, rt_msg);
      bool success_retime = traj_retimer.computeTimeStamps(rt);
      ROS_INFO("Retimed trajectory successfully: %s", success_retime ? "yes" : "no" );
      rt.getRobotTrajectoryMsg(rt_msg);

      // monitor
      AnimationMonitor am;
      ros::Subscriber sub = node_handle.subscribe("animation_status", 1, &AnimationMonitor::statusCallback, &am);

      // set view
      rviz_animated_view_controller::AnimatedViewController avc;

      // publish
      if(1)
      {
        display_trajectory.trajectory_start = mpr_msg.start_state;
        display_trajectory.trajectory.push_back(rt_msg);
        display_publisher.publish(display_trajectory);

        // fork and record
        pid_t pid;
        pid = fork();
        
        if(pid==0)
        {
          // child process records
          system("recordmydesktop -o /tmp/video.ogv");
        }
        else
        {
          // parent spins while the trajectory executes and kills child
          while(ros::ok() && !am.getStatus())
          {
            ros::spinOnce();
          }
          ROS_INFO("Animation almost ready...");
          while(ros::ok() && am.getStatus())
          {
            ros::spinOnce();
          }
          ROS_INFO("Animation terminated");
          kill(0,SIGINT);
        }
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
