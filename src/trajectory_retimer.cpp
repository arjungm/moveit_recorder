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
