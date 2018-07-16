/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Acorn Pooley */

// This code goes with the interactivity tutorial

#include "interactive_robot.h"
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

// default world object position is just in front and left of PR2 robot.
const Eigen::Affine3d InteractiveRobot::DEFAULT_WORLD_OBJECT_POSE_(Eigen::Affine3d(Eigen::Translation3d(0.25, -0.5, 0.5)));

// size of the world geometry cube
const double InteractiveRobot::WORLD_BOX_SIZE_ = 0.15;

// minimum delay between calls to callback function
const ros::Duration InteractiveRobot::min_delay_(0.00001);

InteractiveRobot::InteractiveRobot(const std::string& robot_description,
																	 const std::string& robot_topic,
                                   const std::string& marker_topic,
																	 const std::string& imarker_topic,
																	 const std::string& planning_group,
																	 const std::string& frame_id,
																	 const std::string& name_id,
																	 my_plugin::MyDisplay* pDisplay)
  :  // this node handle is used to create the publishers
  nh_(),

  // load the robot description
  rm_loader_(robot_description),

  // create publishers for markers and robot state
  robot_state_publisher_(nh_.advertise<moveit_msgs::DisplayRobotState>(robot_topic, 1)), 
  world_state_publisher_(nh_.advertise<visualization_msgs::Marker>(marker_topic, 100)),

  // create an interactive marker server for displaying interactive markers
  interactive_marker_server_(imarker_topic), 
	imarker_robot_(0), 
	imarker_world_(0),

	group_(0), 
	user_data_(0), 
	user_callback_(0),
	my_display(pDisplay)
//	robot_state_(robot_state)
{
	ROS_INFO("InteractiveRobot");
	ROS_INFO_STREAM("robot_topic : " << robot_topic);
	ROS_INFO_STREAM("imarker_topic : " << imarker_topic);
	ROS_INFO_STREAM("planning_group : " << planning_group);
	ROS_INFO_STREAM("frame_id : " << frame_id);
	ROS_INFO("InteractiveRobot . . . done");

  // get the RobotModel loaded from urdf and srdf files
  robot_model_ = rm_loader_.getModel();
  if (!robot_model_)
  {
    ROS_ERROR("Could not load robot description");
    throw RobotLoadException();
  }

  // create a RobotState to keep track of the current robot pose
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  if (!robot_state_)
  {
    ROS_ERROR("Could not get RobotState from Model");
    throw RobotLoadException();
  }
  robot_state_->setToDefaultValues();

  // Prepare to move the "right_arm" group
  group_ = robot_state_->getJointModelGroup(planning_group);
  end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);

	color_rgba.r = 255;
	color_rgba.g = 0;
	color_rgba.b = 0;
	color_rgba.a = 1;

  // Create a marker to control the "right_arm" group
  imarker_robot_ = new IMarker(interactive_marker_server_,
															 name_id, 
															 desired_group_end_link_pose_, 
															 frame_id,
                               boost::bind(movedRobotMarkerCallback, this, _1), 
															 IMarker::BOTH);

  // create an interactive marker to control the world geometry (the yellow cube)
/*      desired_world_object_pose_ = DEFAULT_WORLD_OBJECT_POSE_;
  imarker_world_ = new IMarker(interactive_marker_server_, "world", desired_world_object_pose_, "/base_footprint",
                               boost::bind(movedWorldMarkerCallback, this, _1), IMarker::POS),
*/
  // start publishing timer.
  init_time_ = ros::Time::now();
  last_callback_time_ = init_time_;
  average_callback_duration_ = min_delay_;
  schedule_request_count_ = 0;
  publish_timer_ = nh_.createTimer(average_callback_duration_, 
																	 &InteractiveRobot::updateCallback, 
																	 this, 
																	 true);

  // begin publishing robot state
  scheduleUpdate();
}

InteractiveRobot::~InteractiveRobot()
{
  delete imarker_world_;
  delete imarker_robot_;
}

// callback called when marker moves.  Moves right hand to new marker pose.
void InteractiveRobot::movedRobotMarkerCallback(InteractiveRobot* robot,
                                                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
//  Eigen::Affine3d pose;
//  tf::poseMsgToEigen(feedback->pose, pose);
	
  ros::Time t1 = ros::Time::now();
  robot->setGroupPose(feedback->pose);
  ros::Time t2 = ros::Time::now();
	ROS_WARN("InteractiveRobot::movedRobotMarkerCallback() : %lf , %lf", t2.toSec()-t1.toSec(), ((double)(t2.toNSec()-t1.toNSec()))/1000000.0);
}

// callback called when marker moves.  Moves world object to new pose.
void InteractiveRobot::movedWorldMarkerCallback(InteractiveRobot* robot,
                                                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(feedback->pose, pose);
  robot->setWorldObjectPose(pose);
}

// set the callback timer to fire if needed.
// Return true if callback should happen immediately
bool InteractiveRobot::setCallbackTimer(bool new_update_request)
{
  publish_timer_.stop();

  const ros::Time now = ros::Time::now();
  const ros::Duration desired_delay = std::max(min_delay_, average_callback_duration_ * 1.2);
  ros::Duration sec_since_last_callback = now - last_callback_time_;
  ros::Duration sec_til_next_callback = desired_delay - sec_since_last_callback;

  if (schedule_request_count_)
  {
    // need a callback desired_delay seconds after previous callback
    schedule_request_count_ += new_update_request ? 1 : 0;
    if (sec_til_next_callback <= ros::Duration(0.0001))
    {
      // just run the callback now
      return true;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (new_update_request)
  {
    if (sec_til_next_callback < min_delay_)
    {
      // been a while.  Use min_delay_.
      // Set last_callback_time_ to prevent firing too early
      sec_til_next_callback = min_delay_;
      sec_since_last_callback = desired_delay - sec_til_next_callback;
      last_callback_time_ = now - sec_since_last_callback;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (!init_time_.isZero())
  {
    // for the first few seconds after startup call the callback periodically
    // to ensure rviz gets the initial state.
    // Without this rviz does not show some state until markers are moved.
    if ((now - init_time_).sec >= 8)
    {
      init_time_ = ros::Time(0, 0);
      return false;
    }
    else
    {
      publish_timer_.setPeriod(std::max(ros::Duration(1.0), average_callback_duration_ * 2));
      publish_timer_.start();
      return false;
    }
  }
  else
  {
    // nothing to do.  No callback needed.
    return false;
  }
}

// Indicate that the world or the robot has changed and
// the new state needs to be updated and published to rviz
void InteractiveRobot::scheduleUpdate()
{
  //updateAll();

  // schedule an update callback for the future.
  // If the callback should run now, call it.
  if (setCallbackTimer(true))
    updateCallback(ros::TimerEvent());
}

/* callback called when it is time to publish */
void InteractiveRobot::updateCallback(const ros::TimerEvent& e)
{
  ros::Time tbegin = ros::Time::now();
  publish_timer_.stop();

  // do the actual calculations and publishing
  updateAll();

  // measure time spent in callback for rate limiting
  ros::Time tend = ros::Time::now();
  average_callback_duration_ = (average_callback_duration_ + (tend - tbegin)) * 0.5;
  last_callback_time_ = tend;
  schedule_request_count_ = 0;

  // schedule another callback if needed
  setCallbackTimer(false);
}

/* Calculate new positions and publish results to rviz */
void InteractiveRobot::updateAll()
{
  ros::Time t1 = ros::Time::now();
  publishWorldState();

  if (robot_state_->setFromIK(group_, desired_group_end_link_pose_, 10, 0.1))
  {
		ROS_INFO("robot_state_->setFromIK is true");

std::vector<double> ik_seed_state;
robot_state_->copyJointGroupPositions("arm", ik_seed_state);
ROS_INFO("ik_seed_state.size() : %d", ik_seed_state.size());
for(int i=0; i<ik_seed_state.size(); i++)
	ROS_INFO("ik_seed_state[%d] : %lf", i, ik_seed_state[i]);

    publishRobotState();

    if (user_callback_)
      user_callback_(*this);
  }
	else
		ROS_INFO("robot_state_->setFromIK is false");
  ros::Time t4 = ros::Time::now();
	ROS_WARN("InteractiveRobot::updateAll() : %lf , %lf", t4.toSec()-t1.toSec(), ((double)(t4.toNSec()-t1.toNSec()))/1000000.0);
}

// change which group is being manipulated
void InteractiveRobot::setGroup(const std::string& name)
{
  const robot_model::JointModelGroup* group = robot_state_->getJointModelGroup(name);
  if (!group)
  {
    ROS_ERROR_STREAM("No joint group named " << name);
    if (!group_)
      throw RobotLoadException();
  }
  group_ = group;
  std::string end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);
  if (imarker_robot_)
  {
    imarker_robot_->move(desired_group_end_link_pose_);
  }
}

// return current group name
const std::string& InteractiveRobot::getGroupName() const
{
  return group_->getName();
}

/* remember new desired robot pose and schedule an update */
bool InteractiveRobot::setGroupPose(const Eigen::Affine3d& pose)
{
  desired_group_end_link_pose_ = pose;
  scheduleUpdate();
//	this->setGroup(this->getGroupName());
  if (imarker_robot_)
	{
		ROS_INFO("imarker_robot_");
		imarker_robot_->move(pose);
	}
	else
		ROS_INFO("imarker_robot_ is NULL");

}

bool InteractiveRobot::setGroupPose(const geometry_msgs::Pose& pose)
{
  Eigen::Affine3d aff_pose;
  tf::poseMsgToEigen(pose, aff_pose);
  this->setGroupPose(aff_pose);
}

/* publish robot pose to rviz */
void InteractiveRobot::publishRobotState()
{
  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(*robot_state_, msg.state);

	//ROS_INFO("ObjectColor[] highlight_links --> %d", msg.highlight_links.size());

	// Check if a robot state message already exists for this color
	if (msg.highlight_links.size() == 0)  // has not been colored yet, lets create that
	{
		// Get links names
		const std::vector<const moveit::core::LinkModel*>& link_names = robot_state_->getRobotModel()->getLinkModelsWithCollisionGeometry();

		msg.highlight_links.resize(link_names.size());

		// Color every link
		for (std::size_t i = 0; i < link_names.size(); ++i)
		{
			msg.highlight_links[i].id = link_names[i]->getName();
			msg.highlight_links[i].color = color_rgba;
		}
	}

  robot_state_publisher_.publish(msg);
}

/* remember new world object position and schedule an update */
void InteractiveRobot::setWorldObjectPose(const Eigen::Affine3d& pose)
{
  desired_world_object_pose_ = pose;
  scheduleUpdate();
}

/* publish world object position to rviz */
void InteractiveRobot::publishWorldState()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_footprint";
  marker.header.stamp = ros::Time::now();
  marker.ns = "world_box";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = WORLD_BOX_SIZE_;
  marker.scale.y = WORLD_BOX_SIZE_;
  marker.scale.z = WORLD_BOX_SIZE_;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.4f;
  marker.lifetime = ros::Duration();
  tf::poseEigenToMsg(desired_world_object_pose_, marker.pose);
  world_state_publisher_.publish(marker);
}

/* get world object pose and size */
void InteractiveRobot::getWorldGeometry(Eigen::Affine3d& pose, double& size)
{
  pose = desired_world_object_pose_;
  size = WORLD_BOX_SIZE_;
}

std::vector<geometry_msgs::Pose> InteractiveRobot::getPose()
{
	std::vector<geometry_msgs::Pose> v_pose;
  Eigen::Affine3d link;
	geometry_msgs::Pose pose;

	std::string str_pose;
	for(int i=0; i < group_->getLinkModelNames().size(); i++) {
		link = robot_state_->getGlobalLinkTransform(group_->getLinkModelNames()[i]);
		ROS_INFO("link name : %s", group_->getLinkModelNames()[i].c_str());
  	tf::poseEigenToMsg(link, pose);
		v_pose.push_back(pose);
	}
	return v_pose;
}

geometry_msgs::Pose InteractiveRobot::getBackPose()
{
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(desired_group_end_link_pose_, pose);

	return pose;
}

bool InteractiveRobot::setColor(std_msgs::ColorRGBA color)
{
	color_rgba = color;
	return true;
}

std_msgs::ColorRGBA InteractiveRobot::createRandColor()
{
	std_msgs::ColorRGBA result;

	const std::size_t MAX_ATTEMPTS = 20;  // bound the performance
	std::size_t attempts = 0;

	// Make sure color is not *too* dark
	do
	{
		result.r = fRand(0.0, 1.0);
		result.g = fRand(0.0, 1.0);
		result.b = fRand(0.0, 1.0);
		// ROS_DEBUG_STREAM_NAMED(name_, "Looking for random color that is not too light, current value is "
		//<< (result.r + result.g + result.b) << " attempt #" << attempts);
		attempts++;
		if (attempts > MAX_ATTEMPTS)
		{
//			ROS_WARN_STREAM_NAMED(name_, "Unable to find appropriate random color after " << MAX_ATTEMPTS << " attempts");
			break;
		}

	} while (result.r + result.g + result.b < 1.5);  // 3 would be white

	// Set alpha value
	result.a = 1.0;

	return result;
}

float InteractiveRobot::fRand(float dMin, float dMax)
{
	float d = static_cast<float>(rand()) / RAND_MAX;
	return dMin + d * (dMax - dMin);
}

bool InteractiveRobot::setGroupJointPosition(const std::vector<double>& position)
{
	robot_state_->setVariablePositions(position);
}

