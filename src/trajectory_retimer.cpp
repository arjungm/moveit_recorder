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

#include <moveit_recorder/trajectory_retimer.h>

TrajectoryRetimer::TrajectoryRetimer(std::string robot_desc, std::string group_name) : m_traj_retimer(),
   m_psm(robot_desc),
   m_ps(m_psm.getPlanningScene()),
   m_group_name(group_name)   
{
}

TrajectoryRetimer::~TrajectoryRetimer() {}

void TrajectoryRetimer::configure(const moveit_msgs::PlanningScene& ps_msg, 
                             const moveit_msgs::MotionPlanRequest& mpr_msg)
{
  m_ps->setPlanningSceneMsg(ps_msg);
  m_rt = boost::make_shared<robot_trajectory::RobotTrajectory>(m_ps->getRobotModel(), m_group_name);
  m_reference_state = boost::make_shared<moveit::core::RobotState>(m_ps->getRobotModel());
  m_reference_state->setVariableValues(mpr_msg.start_state.joint_state);
}

bool TrajectoryRetimer::retime(moveit_msgs::RobotTrajectory& rt_msg)
{
  m_rt->setRobotTrajectoryMsg(*m_reference_state, rt_msg);
  bool success_retime = m_traj_retimer.computeTimeStamps(*m_rt);
  m_rt->getRobotTrajectoryMsg(rt_msg);
  return success_retime;
}
