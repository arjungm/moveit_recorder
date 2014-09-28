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

#ifndef TRAJ_RETIME_UTIL_H
#define TRAJ_RETIME_UTIL_H

#include <string>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

class TrajectoryRetimer
{
  public:
    TrajectoryRetimer(std::string robot_desc);
    TrajectoryRetimer(std::string robot_desc, std::string group_name);
    ~TrajectoryRetimer();
    void configure(const moveit_msgs::PlanningScene& ps_msg,
                   const moveit_msgs::MotionPlanRequest& mpr_msg);
    bool retime(moveit_msgs::RobotTrajectory& traj);
    void addTimeToStartandGoal(moveit_msgs::RobotTrajectory& rt_msg);
    void correctRootJointPositions();
    void zeroRootJointPositions();
    moveit_msgs::RobotTrajectory createDisplayTrajectoryForState( const moveit_msgs::RobotTrajectory& rt_msg, 
                                                                  const size_t index, const size_t num_waypoints);

    moveit_msgs::RobotState getGoalState(const moveit_msgs::RobotTrajectory& rt_msg);
    moveit_msgs::RobotState getStartState(const moveit_msgs::RobotTrajectory& rt_msg);
    planning_scene::PlanningScenePtr getPlanningScene();
    robot_trajectory::RobotTrajectoryPtr getRobotTrajectory();

  private:
    std::string m_group_name;

    planning_scene_monitor::PlanningSceneMonitor psm_;
    planning_scene::PlanningScenePtr ps_;
    robot_trajectory::RobotTrajectoryPtr rt_;
    moveit_msgs::MotionPlanRequest mpr_;
    moveit::core::RobotStatePtr reference_state_;

    trajectory_processing::IterativeParabolicTimeParameterization traj_retimer_;
};
#endif
