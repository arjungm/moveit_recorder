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
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "moveit_recorder/utils.h"
#include "moveit_recorder/experiment_utils.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uploader_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("user", boost::program_options::value<std::string>(), "Username to upload to")
    ("pass", boost::program_options::value<std::string>(), "Password to upload to")
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
    std::string username = "--email="+utils::get_option(vm, "user", "");
    std::string password = "--password="+utils::get_option(vm, "pass", "");
    std::string category = "--category=Tech";
    std::string save_dir = utils::get_option(vm, "save_dir", "");

    boost::filesystem::path save_directory(save_dir);
    boost::filesystem::path video_bagfile = save_directory / "video_lookup.bag";
    boost::filesystem::path youtube_bagfile = save_directory / "url_lookup.bag";


    // read the bag file to get the file names
    ROS_INFO("Opening bag at %s", video_bagfile.string().c_str());

    rosbag::Bag bag;
    bag.open(video_bagfile.string(), rosbag::bagmode::Read);

    rosbag::View view_all(bag);
    size_t count = 0; 
    std::vector<std::string> list_of_vids;
    std::vector<std::string> list_of_urls;

    std::vector<const rosbag::ConnectionInfo *> connection_infos = view_all.getConnections();
    std::vector<std::string> topics;
    
    BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos)
      topics.push_back(info->topic);
    rosbag::View view_topics(bag, rosbag::TopicQuery(topics));
    
    for( rosbag::View::iterator it=view_topics.begin(); it!=view_topics.end(); ++it)
    {
      std_msgs::String::Ptr strmsg = it->instantiate<std_msgs::String>();
      if(strmsg!=NULL)
      {
        boost::filesystem::path full_topic_name( it->getTopic() );
        boost::filesystem::path video_filepath(strmsg->data);
        std::string video_name = full_topic_name.parent_path().filename().string();
        std::string view_name = full_topic_name.filename().string();

        std::string title = "--title="+video_name+view_name;
        ROS_INFO("Video Title: %s", title.c_str());

        std::string uploader_command = "youtube-upload "+username+" "+password+" "+title+" "+category+" ";
        std::string upload_command = uploader_command + video_filepath.string();
        
        // run the upload
        std::string response_str = utils::system::runCommand( upload_command );

        // check if a youtube link
        if( utils::youtube::isYouTubeLink( response_str ) )
        {
          response_str.erase(std::remove(response_str.begin(), response_str.end(), '\n'), response_str.end());
          // if yes, save to bag under /url/videoname/viewname
          ROS_INFO("Uploaded to %s", response_str.c_str());
          boost::filesystem::path url_topic("/url");
          url_topic =  (url_topic / video_name) / view_name;
          list_of_vids.push_back(url_topic.string());
          list_of_urls.push_back(response_str);
        }
        else
        {
          // if no, retry
          ROS_WARN("Failed to upload the video. No URL detected.");
        }
        count++;
        sleep(2);
      }
    }
    ROS_INFO("Uploaded %d videos in %f seconds", (int)count, 0);
    bag.close();

    rosbag::Bag writeBag;
    writeBag.open(youtube_bagfile.string(), rosbag::bagmode::Write);
    for(int i=0; i<list_of_urls.size(); ++i)
    {
      std_msgs::String url_msg;
      url_msg.data = list_of_urls[i];
      writeBag.write<std_msgs::String>( list_of_vids[i], ros::Time::now(), url_msg );
    }
    writeBag.close();
  }
  catch(...)
  {
    std::cout << "Failure!" << std::endl;
  }

  ROS_INFO("Completed uploading videos.");
  ros::shutdown();

  return 0;
}
