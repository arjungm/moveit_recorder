#include <ros/ros.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit/warehouse/planning_scene_storage.h>

static bool message_set;
static moveit_msgs::PlanningScene current_scene;

void updateRobotStateCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
{
  if(!(msg->is_diff))
  {
    current_scene = *msg;
    message_set = true;
  }
  else
    current_scene.robot_state = msg->robot_state;
}

bool getControlMessage(int input, moveit_msgs::PlanningScene& msg)
{
  moveit_msgs::PlanningScene update_msg(current_scene);
  for(int i=0; i<update_msg.robot_state.multi_dof_joint_state.joint_names.size();++i)
    ROS_INFO("%s", update_msg.robot_state.multi_dof_joint_state.joint_names[i].c_str());
  ROS_INFO("x: %f y: %f z: %f", update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.x,
                                update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.y,
                                update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.z);

  switch(input)
  {
    case 'w':
      update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.x+=0.1;
      ROS_INFO("[BaseControl] Forward");
      goto shared;
    case 'a':
      update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.y+=0.1;
      ROS_INFO("[BaseControl] Left");
      goto shared;
    case 's':
      update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.x-=0.1;
      ROS_INFO("[BaseControl] Backward");
      goto shared;
    case 'd':
      update_msg.robot_state.multi_dof_joint_state.transforms[0].translation.y-=0.1;
      ROS_INFO("[BaseControl] Right");
      shared:
        msg = update_msg;
        return true;
    default:
      ROS_WARN("No command valid");
      break;
  }
  return false;
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
    ("host", boost::program_options::value<std::string>(), "Host for the Mongo database")
    ("port", boost::program_options::value<std::size_t>(), "Port for the Mongo database")
    ("planning_scene_topic", boost::program_options::value<std::string>(), "Planning Scene Topic for the UI")
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
    message_set = false;
    // moveit_warehouse::PlanningSceneStorage pss(host, port);
    // ROS_INFO("Connected to Warehouse DB at host (%s) and port (%d)", host.c_str(), (int)port);

    // planning scene connection for editing in real time
    // publish diffs to change the robot status
    std::string planning_scene_topic = vm.count("planning_scene_topic") ? vm["planning_scene_topic"].as<std::string>() : "planning_scene";
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1);
    ros::Subscriber ps_sub = node_handle.subscribe(planning_scene_topic, 1, updateRobotStateCallback);
    
    while(ps_sub.getNumPublishers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      ROS_INFO("[Designer] Not enough publishers to \"%s\" topic... ", planning_scene_topic.c_str());
      sleep_t.sleep();
    }
    

    while(!message_set)
    {
      ros::spinOnce();
      usleep(1000);
      ROS_WARN("Initial planning scene information not set");
    }

    // spin and catch results
    ROS_INFO("Beginning robot base control loop");
    while(ros::ok())
    {
      ros::spinOnce();
      usleep(1000);

      int key = recorder_utils::getch();
      
      if(key=='q')
        ros::shutdown();

      // process the command
      moveit_msgs::PlanningScene scene_msg;
      bool result = getControlMessage(key, scene_msg);
      if(result)
        ps_pub.publish( scene_msg );
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
  return 0;
}
