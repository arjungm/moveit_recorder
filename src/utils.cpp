#include "moveit_recorder/utils.h"

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

void utils::rosbag_storage::getViewsFromBag(const std::string& bagname, std::vector<view_controller_msgs::CameraPlacement>& views)
{
  rosbag::Bag viewbag;
  viewbag.open(bagname, rosbag::bagmode::Read);
  std::vector<std::string> topics; topics.push_back("viewpoints");
  rosbag::View view_t(viewbag, rosbag::TopicQuery(topics));
  BOOST_FOREACH(rosbag::MessageInstance const m, view_t)
  {
    view_controller_msgs::CameraPlacement::ConstPtr i = m.instantiate<view_controller_msgs::CameraPlacement>();
    if (i != NULL)
      views.push_back(*i);
  }
  viewbag.close();
}

void utils::rostopic::waitOnSubscribersToTopic(const ros::Publisher& pub, const std::string& topic)
{
  while(pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("[Recorder] Not enough subscribers to \"%s\" topic... ", topic.c_str());
    sleep_t.sleep();
  }
}

void utils::rostopic::waitOnPublishersToTopic(const ros::Subscriber& sub, const std::string& topic)
{
  while(sub.getNumPublishers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    ROS_INFO("[Recorder] Not enough publishers to \"%s\" topic... ", topic.c_str());
    sleep_t.sleep();
  }
}
