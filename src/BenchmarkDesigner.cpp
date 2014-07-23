#include <ros/ros.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <iostream>
#include <tf/transform_datatypes.h>

class BaseRobotControl
{
  public:
    BaseRobotControl(ros::NodeHandle nh, 
                     const std::string& planning_scene_topic = "planning_scene",
                     const std::string& from_marker_topic = "from_marker_state",
                     const std::string& to_marker_topic = "to_marker_state")
    : m_node_handle(nh), 
      m_scene_initialized(false), 
      m_planning_scene_topic(planning_scene_topic),
      m_from_marker_topic(from_marker_topic),
      m_to_marker_topic(to_marker_topic)
    {
      // publishers
      m_planning_scene_publisher = m_node_handle.advertise<moveit_msgs::PlanningScene>(m_planning_scene_topic,1);
      m_robot_state_publisher =  m_node_handle.advertise<moveit_msgs::RobotState>(m_to_marker_topic, 1);
      
      // subscribers
      m_planning_scene_subscriber = m_node_handle.subscribe(m_planning_scene_topic, 1,
                                         &BaseRobotControl::planningSceneCallback,
                                         this);
      m_robot_state_subscriber = m_node_handle.subscribe(m_from_marker_topic, 1,
                                         &BaseRobotControl::markerRobotStateCallback,
                                         this);
    }
    ~BaseRobotControl(){}
    void planningSceneCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
    {
      if(!m_scene_initialized)
      {
        m_current_scene = *msg;
        m_current_state = msg->robot_state;
        m_scene_initialized = true;
        m_planning_scene_publisher.publish(m_current_scene); // to rviz & move group
        m_robot_state_publisher.publish(m_current_state); // to interactive robot
      }
    }
    void markerRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg)
    {
      m_current_state = *msg;
      m_current_scene.robot_state = *msg;
      m_planning_scene_publisher.publish(m_current_scene); // to rviz & move group
    }
    void getControlMessage(int dir)
    {
      switch(dir)
      {
        case 'r':
          ROS_INFO("[Reset] Scene is reset");
          m_scene_initialized = false;
          break;
        case 'x':
          ROS_INFO("[Quit] Shutting down");
          ros::shutdown();
        default:
          break;
      }
    }
    bool isSceneInitialized() { return m_scene_initialized; }
    void waitForScene()
    {
      ros::WallDuration sleep_t(1);
      while(!isSceneInitialized())
      {
        ros::spinOnce();
        ROS_WARN("[Designer] No scene set, use MoveIt plugin or publish to \"%s\"", m_planning_scene_topic.c_str());
        sleep_t.sleep();
      }
    }
  private:

    moveit_msgs::PlanningScene m_current_scene;
    moveit_msgs::RobotState m_current_state;

    ros::Publisher m_planning_scene_publisher;
    ros::Publisher m_robot_state_publisher;

    ros::Subscriber m_planning_scene_subscriber;
    ros::Subscriber m_robot_state_subscriber;
    
    ros::NodeHandle m_node_handle;
    
    std::string m_planning_scene_topic;
    std::string m_from_marker_topic;
    std::string m_to_marker_topic;
    
    bool m_scene_initialized;
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
    std::string planning_scene_topic = vm.count("planning_scene_topic") ? 
                                       vm["planning_scene_topic"].as<std::string>() : 
                                       "planning_scene";

    std::string from_marker_topic = vm.count("from_marker_topic") ? 
                                    vm["from_marker_topic"].as<std::string>() : 
                                    "from_marker_state" ;
    std::string to_marker_topic = vm.count("to_marker_topic") ? 
                                  vm["to_marker_topic"].as<std::string>() : 
                                  "to_marker_state" ;

    // initialize the scene and control parser
    BaseRobotControl brc(node_handle, planning_scene_topic, from_marker_topic, to_marker_topic);
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
