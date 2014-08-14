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

#include <ros/ros.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>

#include "moveit_recorder/utils.h"
#include "moveit_recorder/experiment_utils.h"
#include "moveit_recorder/trajectory_video_lookup.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "concat_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("prefix",boost::program_options::value<std::string>(), "Name for merged video")
    ("videos", boost::program_options::value<std::vector<std::string> >()->multitoken(),"named videos")
    ("resources", boost::program_options::value<std::vector<std::string> >()->multitoken(),"resource videos")
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
    // read the bag file to get the file names
    std::string save_dir = utils::get_option(vm, "save_dir", "");
    std::string concat_prefix = utils::get_option(vm, "prefix", "merge");
    boost::filesystem::path save_directory(save_dir);
    TrajectoryVideoLookup video_lookup_table;
    video_lookup_table.loadFromBag( save_directory.string() );
    ROS_INFO("Opening bag at %s", save_directory.string().c_str());
    
    std::vector<std::string> videos = vm["videos"].as<std::vector<std::string> >();
    std::vector<std::string> resources = vm["resources"].as<std::vector<std::string> >();

    ROS_INFO("%d videos and %d resources found", (int)videos.size(), (int)resources.size());

    // iterate over the table and upload the named videos
    TrajectoryVideoLookup::iterator traj_it = video_lookup_table.begin();
    for( ; traj_it!=video_lookup_table.end(); ++traj_it)
    {
      TrajectoryVideoEntry start = video_lookup_table.getNamedVideo( traj_it->first, videos[0] );
      TrajectoryVideoEntry goal = video_lookup_table.getNamedVideo( traj_it->first, videos[1] );
      TrajectoryVideoEntry motion = video_lookup_table.getNamedVideo( traj_it->first, videos[2] );

      // command to merge
      std::string output_file = (save_directory / boost::str(boost::format("%s-%s.%s")  % traj_it->first
                                                                                        % concat_prefix
                                                                                        % "ogv")).string();

      std::string concat_command = boost::str( boost::format( "ffmpeg -i %s -i %s -i %s -i %s -i %s -i %s -filter_complex \'[0:0] [1:0] [2:0] [3:0] [4:0] [5:0] concat=n=6:v=1:a=0 [v] \' -map \'[v]\' -b 20000k %s" ) 
                                                                % resources[0]
                                                                % start.file
                                                                % resources[1]
                                                                % goal.file
                                                                % resources[2]
                                                                % motion.file
                                                                % output_file  );
      std::string response = utils::system::runCommand(concat_command);
      video_lookup_table.putVideoFile(traj_it->first, concat_prefix, output_file);
    }      
    video_lookup_table.saveToBag( save_directory.string() );
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed recording display videos.");
  ros::shutdown();

  return 0;
}


