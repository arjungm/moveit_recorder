#include "moveit_recorder/experiment_utils.h"

#include <iomanip>
#include <iostream>
#include <boost/filesystem.hpp>
#include <cstdio>

std::string utils::system::runCommand(const std::string& command)
{
  char* buffer= new char[255];
  FILE *stream = popen(command.c_str(),"r");
  while ( fgets(buffer, 255, stream) != NULL ) { }
  pclose(stream);
  std::string response_str(buffer);
  delete buffer;

  return response_str;
}

bool utils::youtube::isYouTubeLink(const std::string& str)
{
  if(str.length() < 22)
    return false;
  else
  {
    std::string head = str.substr(0,22);
    if(head=="http://www.youtube.com")
      return true;
    else
      return false;
  }
}

std::string utils::youtube::getYoutubeVideoID(std::string url)
{
  return url.erase(0, url.find_first_of('=')+1);
}

std::string utils::youtube::getYoutubeEmbedURL(const std::string& url)
{
  return "//www.youtube.com/embed/"+getYoutubeVideoID(url);
}


void utils::videoproc::createSplitScreenVideo(std::vector<std::string>& video_lists)
{
}
void utils::rosbag::parseBag()
{
}
std::string utils::rosbag::getTrajectoryName(const std::string& trajectory_topic)
{
  boost::filesystem::path traj_topic_as_path(trajectory_topic);
  return traj_topic_as_path.parent_path().filename().string();
}

