#include <ros/ros.h>

#include <vector>
#include <string>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include "moveit_recorder/utils.h"
#include "moveit_recorder/experiment_utils.h"
#include "moveit_recorder/trajectory_video_lookup.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "split_screen_creator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("prefix", boost::program_options::value<std::string>(), "Prefix of 2x2 merged video")
    ("regex", boost::program_options::value<std::string>(), "Regex of named videos to make 2x2 split screen with")
    ("save_dir",boost::program_options::value<std::string>(), "Directory for videos");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }

  try
  {
    std::string splitscreen_prefix = utils::get_option(vm, "prefix", "split");
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    boost::regex vid_regex(utils::get_option(vm, "regex", "view."));
    boost::filesystem::path save_directory(save_dir);
    
    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );
    
    // create split screen from regex matched videos
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    for(; traj_it!=video_lookup_table.end();++traj_it)
    {
      std::vector<std::string> videos;
      TrajectoryVideoLookupEntry::iterator video_it = traj_it->second.begin();
      for(; video_it!=traj_it->second.end(); ++video_it)
      {
        boost::cmatch matches;
        if(boost::regex_match( video_it->name.c_str(), matches, vid_regex ))
          videos.push_back( video_it->file );
      }
      if(videos.size()!=4)
        ROS_WARN("No videos matching the regex, unable to create split screen video for trajectory id=\"%s\"",traj_it->first.c_str());
      else
      {
        std::string combined = (save_directory / boost::str(boost::format("%s-%s.%s")  % traj_it->first
                                                                                    % splitscreen_prefix
                                                                                    % "ogv")).string();
        std::string combine_command = boost::str(boost::format("ffmpeg -i %s -i %s -i %s -i %s -filter_complex \'[0:v]scale=iw/2:ih/2[rescale1];[1:v]scale=iw/2:ih/2[rescale2];[2:v]scale=iw/2:ih/2[rescale3];[3:v]scale=iw/2:ih/2[rescale4];[rescale1]pad=2*iw:2*ih[topleft];[topleft][rescale2]overlay=W/2:0[topright];[topright][rescale3]overlay=0:H/2[bottomleft];[bottomleft][rescale4]overlay=W/2:H/2[final]\' -map [final] -b 10000k %s") %videos[0] %videos[1] %videos[2] %videos[3] %combined);
        
        std::string response = utils::system::runCommand(combine_command);

        video_lookup_table.putVideoFile(traj_it->first, splitscreen_prefix, combined);
      }
    }
    video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Finished creating splitscreen videos.");
  ros::shutdown();

  return 0;
}
