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

/* Author: Arjun Menon & Acorn Pooley */

#include "moveit_recorder/interactive_robot_marker.h"
#include "moveit_recorder/pose_string.h"
#include <eigen_conversions/eigen_msg.h>


/* default callback which just prints the current pose */
void IMarker::printFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM(feedback->marker_name.c_str()
      << "is now at :"
      << PoseString(feedback->pose));
}


/* create 1 cylinder as part of a 3-cylinder axis (used to visualize pose
 * manipulation) */
visualization_msgs::Marker IMarker::makeAxisCyl( IMarker::Axis axis )
{
  visualization_msgs::Marker marker;
  double scale = imarker_.scale * 0.4;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = scale * 0.15;
  marker.scale.y = scale * 0.15;
  marker.scale.z = scale;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  switch( axis )
  {
    case X:
      marker.pose.position.x = 0.5 * scale;
      marker.pose.orientation.y = 1.0;
      marker.color.r = 0.5;
      break;
    case Y:
      marker.pose.position.y = 0.5 * scale;
      marker.pose.orientation.x = 1.0;
      marker.color.g = 0.5;
      break;
    case Z:
    default:
      marker.pose.position.z = 0.5 * scale;
      marker.color.b = 0.5;
      break;
  }

  return marker;
}

/* create a positional control no marker */
void IMarker::makeBallControl()
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  imarker_.controls.push_back( control );
}

/* create an orientation/position control with an axis marker */
void IMarker::makeAxisControl()
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeAxisCyl(X) );
  control.markers.push_back( makeAxisCyl(Y) );
  control.markers.push_back( makeAxisCyl(Z) );
  imarker_.controls.push_back( control );
}

void IMarker::makePlanarControl()
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeAxisCyl(X) );
  control.markers.push_back( makeAxisCyl(Z) );
  imarker_.controls.push_back( control );
}

void IMarker::setControls(IMarker::Dof dof)
{
  imarker_.controls.clear();
  
  // insert a control with a marker
  switch (dof)
  {
    case POS:
      makeBallControl();
      break;
    case PLANAR:
      makePlanarControl();
      break;
    case ORIENT:
    case BOTH:
    default:
      makeAxisControl();
      break;
  }

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  // add orientation and/or position controls
  for (int i=0; i<3; i++)
  {
    static const char *dirname = "xyz";

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0;
    switch(i) {
      case 0: control.orientation.x = 1; break;
      case 1: control.orientation.y = 1; break;
      case 2: control.orientation.z = 1; break;
    }

    if (dof == ORIENT || dof == BOTH)
    {
      control.name = std::string("rotate_") + std::string(1,dirname[i]);
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      imarker_.controls.push_back(control);
    }
    if (dof == POS || dof == BOTH)
    {
      control.name = std::string("move_") + std::string(1,dirname[i]);
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      imarker_.controls.push_back(control);
    }
    if(dof == PLANAR)
    {
      if(dirname[i]=='y')
      {
        control.name = std::string("rotate_") + std::string(1,dirname[i]);
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      }
      else
      {
        control.name = std::string("move_") + std::string(1,dirname[i]);
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      }
      imarker_.controls.push_back(control);
    }
  }
}

/* move to new pose */
void IMarker::move(const Eigen::Affine3d& pose)
{
  Eigen::Quaterniond q(pose.linear());
  Eigen::Vector3d p = pose.translation();
  tf::pointEigenToMsg(p, imarker_.pose.position);
  tf::quaternionEigenToMsg(q, imarker_.pose.orientation);
  setControls(dof_);
  server_->applyChanges();
}

/* initialize the marker.  All constructors call this. */
void IMarker::initialize(
    interactive_markers::InteractiveMarkerServer& server,
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::string& frame_id,
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> callback,
    IMarker::Dof dof)
{
  server_ = &server;
  imarker_.header.frame_id = frame_id;
  tf::pointEigenToMsg(position, imarker_.pose.position);
  tf::quaternionEigenToMsg(orientation, imarker_.pose.orientation);
  imarker_.scale = 0.3;

  imarker_.name = name;
  imarker_.description = name;

  setControls(dof);

  // tell the server to show the marker and listen for changes
  server_->insert(imarker_);
  server_->setCallback(imarker_.name, callback);
  server_->applyChanges();
}
