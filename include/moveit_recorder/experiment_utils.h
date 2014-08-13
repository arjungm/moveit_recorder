#ifndef EXP_UTILS_H
#define EXP_UTILS_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>

#include <view_controller_msgs/CameraPlacement.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>

namespace utils
{
  namespace system
  {
    std::string runCommand(const std::string& command);
  }

  namespace youtube
  {
    bool isYouTubeLink(const std::string& str);
    std::string getYoutubeVideoID(std::string url);
    std::string getYoutubeEmbedURL(const std::string& url);
  }

  namespace rosbag_storage
  {
    void getViewsFromBag(const std::string& bagname, std::vector<view_controller_msgs::CameraPlacement>& views);
  }
}
#endif
