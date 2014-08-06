#ifndef EXP_UTILS_H
#define EXP_UTILS_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>

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

  namespace videoproc
  {
    void createSplitScreenVideo(std::vector<std::string>& video_lists);
  }

  namespace rosbag
  {
    typedef std::map<std::string, std::vector<std::string> > TrajToVideoMap;
    void parseBag();
    std::string getTrajectoryName(const std::string& trajectory_topic);
  }
}
#endif
