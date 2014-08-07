#include "moveit_recorder/experiment_utils.h"

#include <iomanip>
#include <iostream>
#include <cstdio>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

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
  //TODO check is valid youtube URL
  return url.erase(0, url.find_first_of('=')+1);
}

std::string utils::youtube::getYoutubeEmbedURL(const std::string& url)
{
  return "//www.youtube.com/embed/"+getYoutubeVideoID(url);
}


void utils::videoproc::createSplitScreenVideo(std::vector<std::string>& video_lists)
{
}
