#ifndef INTERACT_ROBOT_H
#define INTERACT_ROBOT_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include "moveit_recorder/InteractiveRobotMarker.h"


/** Keeps track of the state of the robot and the world.
 * Updates the state when interactive markers are manipulated.
 * Publishes the state to rviz. */
class InteractiveRobot {
  public:
    InteractiveRobot(
        const std::string& robot_description = "robot_description",
        const std::string& planning_scene_topic = "planning_scene",
        const std::string& from_scene_topic = "to_marker_state",
        const std::string& to_scene_topic = "from_marker_state",
        const std::string& to_scene_pose_topic = "from_marker_pose",
        const std::string& display_robot_topic = "interactive_robot_state",
        const std::string& marker_topic = "interactive_robot_markers",
        const std::string& imarker_topic = "interactive_robot_imarkers",
        const std::string& arm_name = "right_arm");
    ~InteractiveRobot();

    /** set which group to manipulate */
    void setGroup(const std::string& name);
    const std::string& getGroupName() const;

    /** Set the pose of the group we are manipulating */
    bool setGroupPose(const Eigen::Affine3d& pose);
    bool setOtherGroupPose(const Eigen::Affine3d& pose);
    bool setBasePose(double x, double y, double theta);

    /** set a callback to call when updates occur */
    void setUserCallback(boost::function<void (InteractiveRobot& robot)> callback)
    {
      user_callback_ = callback;
    }

    /** access RobotModel */
    robot_model::RobotModelPtr& robotModel() { return robot_model_; }
    /** access RobotState */
    robot_state::RobotStatePtr& robotState() { return robot_state_; }

    /** exception thrown when a problem occurs */
    class RobotLoadException : std::exception
  { };

    /** hook for user data.  Unused by the InteractiveRobot class.
     * initialized to 0 */
    void *user_data_;

  private:
    /* Indicate that the world or the robot has changed and
     * the new state needs to be updated and published to rviz */
    void scheduleUpdate();


    /* set the callback timer to fire if needed.
     * Return true if callback should happen immediately. */
    bool setCallbackTimer(bool new_update_request);

    /* update the world and robot state and publish to rviz */
    void updateCallback(const ros::TimerEvent& e);

    /* functions to calculate new state and publish to rviz */
    void updateAll();
    void publishRobotState();

    /* callback called when marker moves.  Moves right hand to new marker pose. */
    static void movedRobotMarkerCallback(
        InteractiveRobot *robot,
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    static void movedOtherMarkerCallback(
        InteractiveRobot *robot,
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    static void movedRobotBaseMarkerCallback(
        InteractiveRobot *robot,
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    
    void updateRobotStateCallback(const boost::shared_ptr<moveit_msgs::RobotState const>& msg);
    void resetMarkers();

    /* marker publishers */
    ros::NodeHandle nh_;
    ros::Subscriber robot_state_subscriber_;
    ros::Publisher robot_state_publisher_;
    ros::Publisher marker_robot_state_publisher_;
    ros::Publisher marker_robot_pose_publisher_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
    IMarker *imarker_robot_;
    IMarker *imarker_other_;
    IMarker *imarker_base_;

    /* robot info */
    robot_model_loader::RobotModelLoader rm_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    /* info about joint group we are manipulating */
    robot_state::JointModelGroup* group_;
    robot_state::JointModelGroup* other_group_;
    const robot_state::JointModel* base_joint_;
    Eigen::Affine3d desired_group_end_link_pose_;
    Eigen::Affine3d other_desired_group_end_link_pose_;
    Eigen::Affine3d desired_base_link_pose_;
    
    Eigen::Affine3d pose_of_ee_in_base_;
    Eigen::Affine3d other_pose_of_ee_in_base_;

    std::string end_link_;
    std::string other_end_link_;
    std::string arm_name_;
    std::string other_arm_name_;

    /* user callback function */
    boost::function<void (InteractiveRobot& robot)> user_callback_;

    /* timer info for rate limiting */
    ros::Timer publish_timer_;
    ros::Time init_time_;
    ros::Time last_callback_time_;
    ros::Duration average_callback_duration_;
    static const ros::Duration min_delay_;
    int schedule_request_count_;
    bool robot_state_initialized_;
    bool base_changed_;
};

#endif
