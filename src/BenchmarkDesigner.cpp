#include <ros/ros.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <curses.h>

class BaseRobotControl
{
  public:
    BaseRobotControl(){}
    ~BaseRobotControl(){}
    void updateRobotStateCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
    {
      m_current_scene = *msg;
    }
    moveit_msgs::PlanningScene getControlMessage(int dir)
    {
      moveit_msgs::PlanningScene copy_msg(m_current_scene);
      switch(dir)
      {
        case 'w':
          copy_msg.robot_state.multi_dof_joint_state.transforms[0].translation.x+=0.1;
          printw("[Forward]\n");
          break;
        case 'a':
          copy_msg.robot_state.multi_dof_joint_state.transforms[0].translation.y+=0.1;
          printw("[Left]\n");
          break; 
        case 's':
          copy_msg.robot_state.multi_dof_joint_state.transforms[0].translation.x-=0.1;
          printw("[Backward]\n");
          break;
        case 'd':
          copy_msg.robot_state.multi_dof_joint_state.transforms[0].translation.y-=0.1;
          printw("[Right]\n");
          break;
        default:
          break;
      }
      return copy_msg;
    }
  private:
    moveit_msgs::PlanningScene m_current_scene;
};

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
    // moveit_warehouse::PlanningSceneStorage pss(host, port);
    // ROS_INFO("Connected to Warehouse DB at host (%s) and port (%d)", host.c_str(), (int)port);

    // planning scene connection for editing in real time
    // publish diffs to change the robot status
    BaseRobotControl brc;
    std::string planning_scene_topic = vm.count("planning_scene_topic") ? vm["planning_scene_topic"].as<std::string>() : "planning_scene";
    ros::Publisher ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 1);
    ros::Subscriber ps_sub = node_handle.subscribe(planning_scene_topic, 1, &BaseRobotControl::updateRobotStateCallback, &brc);

    
    // spin and catch results
    initscr();
    noecho();
    cbreak();
    timeout(0);
    int n=0;
    while(ros::ok())
    {
      ros::spinOnce();
      usleep(1000);
      
      int key = getch();
      
      // process the command
      moveit_msgs::PlanningScene update_scene = brc.getControlMessage(key);
      ps_pub.publish( update_scene );
    }
    nocbreak();
    endwin();
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
