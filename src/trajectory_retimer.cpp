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

#include <moveit/robot_state/conversions.h>
#include <moveit_recorder/trajectory_retimer.h>

TrajectoryRetimer::TrajectoryRetimer(std::string robot_desc) : traj_retimer_(),
   psm_(robot_desc),
   ps_(psm_.getPlanningScene())
{
}

TrajectoryRetimer::TrajectoryRetimer(std::string robot_desc, std::string group_name) : traj_retimer_(),
   psm_(robot_desc),
   ps_(psm_.getPlanningScene()),
   m_group_name(group_name)   
{
}

TrajectoryRetimer::~TrajectoryRetimer() {}

void TrajectoryRetimer::configure(const moveit_msgs::PlanningScene& ps_msg, 
                             const moveit_msgs::MotionPlanRequest& mpr_msg)
{
  ps_->setPlanningSceneMsg(ps_msg);
  m_group_name = mpr_msg.group_name;
  rt_ = boost::make_shared<robot_trajectory::RobotTrajectory>(ps_->getRobotModel(), m_group_name);
  reference_state_ = boost::make_shared<moveit::core::RobotState>(ps_->getRobotModel());
  reference_state_->setVariableValues(mpr_msg.start_state.joint_state);
}

bool TrajectoryRetimer::retime(moveit_msgs::RobotTrajectory& rt_msg)
{
  rt_->setRobotTrajectoryMsg(*reference_state_, rt_msg);
  bool success_retime = traj_retimer_.computeTimeStamps(*rt_);
  rt_->getRobotTrajectoryMsg(rt_msg);
  return success_retime;
}

planning_scene::PlanningScenePtr TrajectoryRetimer::getPlanningScene()
{
  return ps_;
}

robot_trajectory::RobotTrajectoryPtr TrajectoryRetimer::getRobotTrajectory()
{
  return rt_;
}


void TrajectoryRetimer::addTimeToStartandGoal(moveit_msgs::RobotTrajectory& rt_msg)
{
  // modify robot trajectory for longer start and stop visualization.
  rt_->setRobotTrajectoryMsg(*reference_state_, rt_msg);
  const robot_state::RobotState first = rt_->getFirstWayPoint();
  const robot_state::RobotState last = rt_->getLastWayPoint();

  rt_->setWayPointDurationFromPrevious(0,1); // add 2s to first way point
  for(int i=0; i<5; i++)
    rt_->addPrefixWayPoint(first, 1); // repeat the first waypoint so it visualizes start for 2s
  for(int i=0; i<5; i++)
    rt_->addSuffixWayPoint(last, 1); // repeat the last waypoint for 2s
  
  rt_->getRobotTrajectoryMsg(rt_msg);
}

moveit_msgs::RobotState TrajectoryRetimer::getStartState(const moveit_msgs::RobotTrajectory& rt_msg)
{
  moveit_msgs::RobotState display_rs_msg;
  rt_->setRobotTrajectoryMsg(*reference_state_, rt_msg);
  const robot_state::RobotState state = rt_->getWayPoint(0);
  robotStateToRobotStateMsg(state, display_rs_msg);
  return display_rs_msg;
}
moveit_msgs::RobotState TrajectoryRetimer::getGoalState(const moveit_msgs::RobotTrajectory& rt_msg)
{
  moveit_msgs::RobotState display_rs_msg;
  rt_->setRobotTrajectoryMsg(*reference_state_, rt_msg);
  const robot_state::RobotState state = rt_->getWayPoint( rt_msg.joint_trajectory.points.size()-1 );
  robotStateToRobotStateMsg(state, display_rs_msg);
  return display_rs_msg;
}

moveit_msgs::RobotTrajectory TrajectoryRetimer::createDisplayTrajectoryForState(const moveit_msgs::RobotTrajectory& rt_msg, const size_t index, const size_t num_waypoints)
{
  // create display trajectory
  moveit_msgs::RobotTrajectory display_rt_msg;
  robot_trajectory::RobotTrajectoryPtr display_rt = boost::make_shared<robot_trajectory::RobotTrajectory>(ps_->getRobotModel(), m_group_name);
  
  // get way point
  rt_->setRobotTrajectoryMsg(*reference_state_, rt_msg);
  const robot_state::RobotState state = rt_->getWayPoint( index );
  
  // populate display trajectory
  for(int i=0; i<num_waypoints; i++)
    display_rt->addSuffixWayPoint(state, 1); // repeat the last waypoint for 2s
  
  // return as message
  display_rt->getRobotTrajectoryMsg(display_rt_msg);
  return display_rt_msg;
}
