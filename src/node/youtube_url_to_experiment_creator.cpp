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
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "moveit_recorder/utils.h"

using namespace std;

typedef std::map<std::string, std::vector<std::string> > TrajToVideoMap;

std::string getTrajectoryName(const std::string& trajectory_topic)
{
  boost::filesystem::path traj_topic_as_path(trajectory_topic);
  return traj_topic_as_path.parent_path().filename().string();
}

std::string getYoutubeVideoID(std::string url)
{
  return url.erase(0, url.find_first_of('=')+1);
}

std::string getYoutubeEmbedURL(const std::string& url)
{
  return "//www.youtube.com/embed/"+getYoutubeVideoID(url);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youtube_url_to_experiment_creator");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
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
    boost::filesystem::path save_directory(save_dir);
    boost::filesystem::path youtube_bagfile = save_directory / "url_lookup.bag";
    boost::filesystem::path experiment_file = save_directory / "experiment.csv";

    TrajToVideoMap traj_video_map;

    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", youtube_bagfile.string().c_str());

    rosbag::Bag bag;
    bag.open(youtube_bagfile.string(), rosbag::bagmode::Read);
    
    size_t count = 0; 
    
    // get all topics in bag
    rosbag::View view_all(bag);
    std::vector<const rosbag::ConnectionInfo *> connection_infos = view_all.getConnections();
    std::vector<std::string> topics;
    BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos)
      topics.push_back(info->topic);
    rosbag::View view_topics(bag, rosbag::TopicQuery(topics));
    
    // collect all the topics and video urls into a nice map
    size_t min_num_tags = 0;
    for( rosbag::View::iterator it=view_topics.begin(); it!=view_topics.end(); ++it)
    {
      std_msgs::String::Ptr strmsg = it->instantiate<std_msgs::String>();
      if(strmsg!=NULL)
      {
        std::string trajectory_name = getTrajectoryName(it->getTopic());
        TrajToVideoMap::iterator got = traj_video_map.find(trajectory_name);
        
        // has newlines ... need to catch this earlier
        std::string str = strmsg->data;
        str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());

        if(got!=traj_video_map.end())
        {
          // add url to list of urls for this traj
          got->second.push_back( getYoutubeEmbedURL(str) );
          min_num_tags = got->second.size() > min_num_tags ? got->second.size() : min_num_tags;
        }
        else
        {
          // if traj, create new url list
          std::vector<std::string> list_of_urls;
          list_of_urls.push_back( getYoutubeEmbedURL(str) );
          traj_video_map.insert( std::make_pair<std::string, 
                                                std::vector<std::string> >
                                                (trajectory_name, list_of_urls) );
        }
      }
    }
    bag.close();

    // iterate over the keys in the map and save to CSV
    ROS_INFO("Saving to experiment file: %s", experiment_file.string().c_str());
    std::ofstream file;
    file.open(experiment_file.string().c_str());
    
    ROS_INFO("Minumum number of tags: %d", (int)min_num_tags);
    // tag line for amazon exp file is "video1, video2, ... videoN"
    for(size_t i=0; i<min_num_tags; ++i)
    {
      file << "video";
      file << (i+1);
      file << ((i+1) < min_num_tags?",":"\n");
    }

    TrajToVideoMap::iterator it;
    for( it=traj_video_map.begin(); it!=traj_video_map.end(); ++it)
    {
      assert( it->second.size() <= min_num_tags);
      if(it->second.size() < min_num_tags)
      {
        ROS_WARN("Not enough videos for the number of tags");
        // not enough videos for the tags, pad the remaining with empty strings
        for(size_t i=0; i<it->second.size();++i)
          file << it->second[i] << ",";
        for(size_t i=it->second.size(); i<min_num_tags;++i)
          file << (((i+1)<min_num_tags)?",":"\n");
      }
      else
      {
        // same number of videos 
        for(size_t k=0; k<min_num_tags;++k)
        {
          file << it->second[k];
          std::string add = (((k+1)<min_num_tags)?",":"\n");
          file << add;
        }
      }
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

