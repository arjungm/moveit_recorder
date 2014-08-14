#ifndef RECORDER_UTILS_H
#define RECORDER_UTILS_H

#include <string>
#include <view_controller_msgs/CameraPlacement.h>
#include <boost/program_options.hpp>

namespace utils
{
  inline std::string get_option(const boost::program_options::variables_map& vm, const std::string& option, const std::string& default_str)
  {
    return vm.count(option) ? vm[option].as<std::string>() : default_str;
  }
  namespace rosbag_storage
  {
    void getViewsFromBag(const std::string& bagname, std::vector<view_controller_msgs::CameraPlacement>& views);
  }
}
#endif
