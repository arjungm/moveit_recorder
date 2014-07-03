#include <string>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>


#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PlanningScene.h>


#ifndef TRAJ_RETIME_UTIL_H
#define TRAJ_RETIME_UTIL_H

class TrajectoryRetimer
{
  public:
    TrajectoryRetimer(std::string robot_desc, std::string group_name);
    ~TrajectoryRetimer();
    void configure(const moveit_msgs::PlanningScene& ps_msg,
                   const moveit_msgs::MotionPlanRequest& mpr_msg);
    bool retime(moveit_msgs::RobotTrajectory& traj);

  private:
    std::string m_group_name;

    planning_scene_monitor::PlanningSceneMonitor m_psm;
    planning_scene::PlanningScenePtr m_ps;
    robot_trajectory::RobotTrajectoryPtr m_rt;
    moveit::core::RobotStatePtr m_reference_state;

    trajectory_processing::IterativeParabolicTimeParameterization m_traj_retimer;
};
#endif
