/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Willow Garage, Inc. 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Arjun Menon */

#include <map>
#include <fstream>
#include <iostream>

#include <ros/ros.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include "moveit_recorder/utils.h"
#include "moveit_recorder/experiment_utils.h"
#include "moveit_recorder/trajectory_video_lookup.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youtube_url_to_experiment_creator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("regex", boost::program_options::value<std::string>(), "Regex of named videos to experiment with")
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
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    boost::regex vid_regex(utils::get_option(vm, "regex", "view."));
    boost::filesystem::path save_directory(save_dir);
    boost::filesystem::path experiment_file = save_directory / "experiment.csv";

    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );

    // iterate over the keys in the map and save to CSV
    ROS_INFO("Saving to experiment file: %s", experiment_file.string().c_str());
    std::ofstream file;
    file.open(experiment_file.string().c_str());
    
    // count the number of videos
    size_t min_num_tags = 0;
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    for(; traj_it!=video_lookup_table.end();++traj_it)
    {
      size_t num_videos=0;
      TrajectoryVideoLookupEntry::iterator video_it = traj_it->second.begin();
      for(; video_it!=traj_it->second.end();++video_it)
      {
        boost::cmatch matches;
        if(boost::regex_match( video_it->name.c_str(), matches, vid_regex ))
        {
          num_videos++;
        }
      }
      min_num_tags = (min_num_tags<num_videos)?num_videos:min_num_tags;
    }
    ROS_INFO("Minimum number of experiment tags need %d", (int) min_num_tags);

    // tag line for amazon exp file is "video1, video2, ... videoN"
    for(size_t i=0; i<min_num_tags; ++i)
    {
      file << "video";
      file << (i+1);
      file << ((i+1) < min_num_tags?",":"\n");
    }

    bool put_newline = false;
    traj_it = video_lookup_table.begin();
    for(; traj_it!=video_lookup_table.end();++traj_it)
    {
      if(put_newline)
        file << std::endl;
      
      size_t num_videos=0;
      TrajectoryVideoLookupEntry::iterator video_it = traj_it->second.begin();

      while( num_videos<min_num_tags && video_it!=traj_it->second.end() )
      {
        boost::cmatch matches;
        if(boost::regex_match( video_it->name.c_str(), matches, vid_regex ))
        {
          std::string url = utils::youtube::getYoutubeEmbedURL(video_it->url);
          file << url;
          if(num_videos < min_num_tags-1)
            file << ",";
          num_videos++;
        }
        video_it++;
        if(video_it==traj_it->second.end())
          video_it = traj_it->second.begin();
      }
      put_newline=true;
    }
    file.close();
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed creating experiment csv.");
  ros::shutdown();

  return 0;
}

