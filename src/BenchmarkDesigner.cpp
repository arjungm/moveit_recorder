#include <ros/ros.h>
#include <moveit_recorder/cli_controller.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <iostream>
#include <tf/transform_datatypes.h>

class BaseRobotControl
{
  public:
    BaseRobotControl(ros::NodeHandle nh, std::string planning_scene_topic) 
    : m_node_handle(nh), 
      m_scene_initialized(false), 
      m_publish(true),
      m_planning_scene_topic(planning_scene_topic)
    {
      m_ps_pub = m_node_handle.advertise<moveit_msgs::PlanningScene>(m_planning_scene_topic,1);
    }
    ~BaseRobotControl(){}
    void updateRobotStateCallback(const boost::shared_ptr<moveit_msgs::PlanningScene const>& msg)
    {
      if(!m_scene_initialized)
      {
        m_current_scene = *msg;
        m_scene_initialized = true;
        //update yaw
        tf::Quaternion qt;
        tf::quaternionMsgToTF( m_current_scene.robot_state.multi_dof_joint_state.transforms[0].rotation, qt);
        m_yaw = tf::getYaw(qt);
      }
      if(m_publish)
        m_ps_pub.publish(m_current_scene);
    }
    moveit_msgs::PlanningScene getControlMessage(int dir)
    {
      switch(dir)
      {
        case 'w':
          m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.x+=0.1;
          ROS_INFO("[Forward] x=%f y=%f",
                              m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.x,
                              m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.y);
          break;
        case 'a':
          m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.y+=0.1;
          ROS_INFO("[Left] x=%f y=%f",
                           m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.x,
                           m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.y);
         break; 
        case 's':
          m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.x-=0.1;
          ROS_INFO("[Backward] x=%f y=%f",
                               m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.x,
                               m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.y);
          break;
        case 'd':
          m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.y-=0.1;
          ROS_INFO("[Right] x=%f y=%f",
                            m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.x,
                            m_current_scene.robot_state.multi_dof_joint_state.transforms[0].translation.y);
          break;
        case 'q':
          m_yaw += M_PI/8;
          m_current_scene.robot_state.multi_dof_joint_state.transforms[0].rotation = tf::createQuaternionMsgFromYaw(m_yaw);
          ROS_INFO("[Turn] Right by %f", M_PI/8);
          break;
        case 'e':
          m_yaw -= M_PI/8;
          m_current_scene.robot_state.multi_dof_joint_state.transforms[0].rotation = tf::createQuaternionMsgFromYaw(m_yaw);
          ROS_INFO("[Turn] Right by %f", M_PI/8);
          break;
        case 'r':
          ROS_INFO("[Reset] Scene is reset");
          m_scene_initialized = false;
          break;
        case 'p':
          ROS_INFO("[Toggle] Publishing is %s", !m_publish?"ON":"OFF");
          m_publish = !m_publish;
          break;
        case 'x':
          ROS_INFO("[Quit] Shutting down");
          ros::shutdown();
        default:
          break;
      }
      return m_current_scene;
    }
    bool isSceneInitialized() { return m_scene_initialized; }
    void waitForScene()
    {
      ros::WallDuration sleep_t(0.5);
      while(!isSceneInitialized())
      {
        ros::spinOnce();
        ROS_WARN("[Designer] No scene is set, please load a scene from the RVIZ MoveIt plugin or publish it to the topic: \"%s\"", m_planning_scene_topic.c_str());
        sleep_t.sleep();
      }
    }
  private:
    moveit_msgs::PlanningScene m_current_scene;
    ros::Publisher m_ps_pub;
    ros::NodeHandle m_node_handle;
    std::string m_planning_scene_topic;
    double m_yaw;
    bool m_scene_initialized;
    bool m_publish;
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
    std::string planning_scene_topic = vm.count("planning_scene_topic") ? vm["planning_scene_topic"].as<std::string>() : "planning_scene";

    // initialize the scene and control parser
    BaseRobotControl brc(node_handle, planning_scene_topic);
    ros::Subscriber ps_sub = node_handle.subscribe(planning_scene_topic, 1, &BaseRobotControl::updateRobotStateCallback, &brc);
    brc.waitForScene();
    
    // spin and catch results
    moveit_msgs::PlanningScene update_scene;
    while(ros::ok())
    {
      ros::spinOnce();
      usleep(1000);
      
      char key;
      std::cin >> key;
      
      // process the command
      update_scene = brc.getControlMessage(key);

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
