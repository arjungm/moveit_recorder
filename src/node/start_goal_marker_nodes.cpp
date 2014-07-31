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
#include "moveit_recorder/utils.h"
#include "moveit_recorder/interactive_robot.h"

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "start_marker");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(20);
  
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("robot_description", boost::program_options::value<std::string>(), "robot description param name")
    ("planning_scene_topic", boost::program_options::value<std::string>(), "planning scene topic")
    ("to_marker_topic", boost::program_options::value<std::string>(), "robot state topic FROM planning scene")
    ("from_marker_topic", boost::program_options::value<std::string>(), "robot state topic TO planning scene")
    ("from_marker_pose_topic", boost::program_options::value<std::string>(), "pose topic for robot end link pose")
    ("display_robot_topic", boost::program_options::value<std::string>(), "display robot state topic")
    ("robot_marker_topic", boost::program_options::value<std::string>(), "topic for robot visual markers")
    ("interactive_marker_topic", boost::program_options::value<std::string>(), "topic for interactive marker");

  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }
  try
  {
    std::string robot_description = utils::get_option(vm, "robot_description", "robot_description");
    std::string planning_scene_topic = utils::get_option(vm, "planning_scene_topic", "planning_scene");
    std::string to_marker_topic = utils::get_option(vm, "to_marker_topic", "to_marker_state");
    std::string from_marker_topic = utils::get_option(vm, "from_marker_topic", "from_marker_state");
    std::string from_marker_pose_topic = utils::get_option(vm, "from_marker_pose_topic", "from_marker_pose");
    std::string display_robot_topic = utils::get_option(vm, "display_robot_topic", "interactive_robot_state");
    std::string robot_marker_topic = utils::get_option(vm, "robot_marker_topic", "interactive_robot_markers");
    std::string interactive_marker_topic = utils::get_option(vm, "interactive_marker_topic", "interactive_robot_imarkers");

    InteractiveRobot robot( robot_description,
                            planning_scene_topic,
                            to_marker_topic,
                            from_marker_topic,
                            from_marker_pose_topic,
                            display_robot_topic,
                            robot_marker_topic,
                            interactive_marker_topic,
                            "right_arm");
    ros::spin();
  }
  catch(...)
  {

  }

  ros::shutdown();
  return 0;
}
