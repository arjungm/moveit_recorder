#include "moveit_recorder/InteractiveRobot.h"
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>


// minimum delay between calls to callback function
const ros::Duration InteractiveRobot::min_delay_(0.01);

InteractiveRobot::InteractiveRobot(
    const std::string& robot_description,
    const std::string& robot_topic,
    const std::string& marker_topic,
    const std::string& imarker_topic) :
  // this node handle is used to create the publishers
  nh_(),
  // create publishers for markers and robot state
  robot_state_publisher_(nh_.advertise<moveit_msgs::DisplayRobotState>(robot_topic,1)),
  // create an interactive marker server for displaying interactive markers
  interactive_marker_server_(imarker_topic),
  imarker_robot_(0),
  // load the robot description
  rm_loader_(robot_description),
  group_(0),
  user_data_(0),
  user_callback_(0)
{
  // get the RobotModel loaded from urdf and srdf files
  robot_model_ = rm_loader_.getModel();
  if (!robot_model_) {
    ROS_ERROR("Could not load robot description");
    throw RobotLoadException();
  }

  // create a RobotState to keep track of the current robot pose
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  if (!robot_state_) {
    ROS_ERROR("Could not get RobotState from Model");
    throw RobotLoadException();
  }
  robot_state_->setToDefaultValues();

  // Prepare to move the "right_arm" group
  group_ = robot_model_->getJointModelGroup("right_arm");
  std::string end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);
  
  // Create a marker to control the "right_arm" group
  imarker_robot_ = new IMarker(
      interactive_marker_server_,
      "robot",
      desired_group_end_link_pose_,
      "/base_footprint",
      boost::bind(movedRobotMarkerCallback,this,_1),
      IMarker::BOTH);


  // start publishing timer.
  init_time_ = ros::Time::now();
  last_callback_time_ = init_time_;
  average_callback_duration_ = min_delay_;
  schedule_request_count_ = 0;
  publish_timer_ = nh_.createTimer(average_callback_duration_, &InteractiveRobot::updateCallback, this, true);

  // begin publishing robot state
  scheduleUpdate();
}

InteractiveRobot::~InteractiveRobot()
{
  delete imarker_robot_;
}

// callback called when marker moves.  Moves right hand to new marker pose.
void InteractiveRobot::movedRobotMarkerCallback(
    InteractiveRobot *robot,
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(feedback->pose, pose);
  robot->setGroupPose(pose);
}

// set the callback timer to fire if needed.
// Return true if callback should happen immediately
bool InteractiveRobot::setCallbackTimer(bool new_update_request)
{
  publish_timer_.stop();

  const ros::Time now = ros::Time::now();
  const ros::Duration desired_delay = std::max(min_delay_, average_callback_duration_ * 1.2);
  ros::Duration sec_since_last_callback = now - last_callback_time_;
  ros::Duration sec_til_next_callback = desired_delay - sec_since_last_callback;

  if (schedule_request_count_)
  {
    // need a callback desired_delay seconds after previous callback
    schedule_request_count_ += new_update_request ? 1 : 0;
    if (sec_til_next_callback <= ros::Duration(0.0001))
    {
      // just run the callback now
      return true;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (new_update_request)
  {
    if (sec_til_next_callback < min_delay_) {
      // been a while.  Use min_delay_.
      // Set last_callback_time_ to prevent firing too early
      sec_til_next_callback = min_delay_;
      sec_since_last_callback = desired_delay - sec_til_next_callback;
      last_callback_time_ = now - sec_since_last_callback;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (!init_time_.isZero())
  {
    // for the first few seconds after startup call the callback periodically
    // to ensure rviz gets the initial state.
    // Without this rviz does not show some state until markers are moved.
    if ((now - init_time_).sec >= 8)
    {
      init_time_ = ros::Time(0,0);
      return false;
    }
    else
    {
      publish_timer_.setPeriod(std::max(ros::Duration(1.0), average_callback_duration_*2));
      publish_timer_.start();
      return false;
    }
  }
  else
  {
    // nothing to do.  No callback needed.
    return false;
  }
}

// Indicate that the world or the robot has changed and
// the new state needs to be updated and published to rviz
void InteractiveRobot::scheduleUpdate()
{
  // schedule an update callback for the future.
  // If the callback should run now, call it.
  if (setCallbackTimer(true))
    updateCallback(ros::TimerEvent());
}


/* callback called when it is time to publish */
void InteractiveRobot::updateCallback(const ros::TimerEvent& e)
{
  ros::Time tbegin = ros::Time::now();
  publish_timer_.stop();

  // do the actual calculations and publishing
  updateAll();

  // measure time spent in callback for rate limiting
  ros::Time tend = ros::Time::now();
  average_callback_duration_ = (average_callback_duration_ + (tend - tbegin)) * 0.5;
  last_callback_time_ = tend;
  schedule_request_count_ = 0;

  // schedule another callback if needed
  setCallbackTimer(false);
}

/* Calculate new positions and publish results to rviz */
void InteractiveRobot::updateAll()
{
  //TODO get const *
  if (robot_state_->setFromIK(robot_state_->getJointModelGroup("right_arm"), 
                              desired_group_end_link_pose_, 10, 0.1))
  {
    publishRobotState();

    if (user_callback_)
      user_callback_(*this);
  }
}

// change which group is being manipulated
void InteractiveRobot::setGroup(const std::string& name)
{
  robot_state::JointModelGroup* group = robot_model_->getJointModelGroup(name);
  if (!group)
  {
    ROS_ERROR_STREAM("No joint group named " << name);
    if (!group_)
      throw RobotLoadException();
  }
  group_ = group;
  std::string end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);
  if (imarker_robot_)
  {
    imarker_robot_->move(desired_group_end_link_pose_);
  }
}

// return current group name
const std::string& InteractiveRobot::getGroupName() const
{
  return group_->getName();
}

/* remember new desired robot pose and schedule an update */
bool InteractiveRobot::setGroupPose(const Eigen::Affine3d& pose)
{
  desired_group_end_link_pose_ = pose;
  scheduleUpdate();
}

/* publish robot pose to rviz */
void InteractiveRobot::publishRobotState()
{
  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(*robot_state_, msg.state);
  robot_state_publisher_.publish(msg);
}
