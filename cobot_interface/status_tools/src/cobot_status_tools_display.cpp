#include <cobot_status_tools_display.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/bool_property.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <QtMath>
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "cobot_msgs/ReadJointStateFile.h"
#include "cobot_msgs/Jog.h"
#include "moveit_msgs/GetPositionFK.h"
#include "cobot_msgs/ReadJointStateFile.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "cobot_planner/CobotPlanning.h"

#include "ui_cobot_status_tools.h"

using namespace cobot_interface;

CobotStatusToolsDisplay *md = NULL;

CobotStatusToolsDisplay::CobotStatusToolsDisplay() : nh_(), spinner(1)
{
	ROS_WARN("CobotStatusToolsDisplay constructor");
	Display();
	frame_ = NULL;
	frame_dock_ = NULL;
	spinner.start();
}

CobotStatusToolsDisplay::~CobotStatusToolsDisplay()
{
	ROS_INFO("CobotStatusToolsDisplay::~CobotStatusToolsDisplay()");
	sub_js.shutdown();
	if (frame_dock_)
    delete frame_dock_;
}

void CobotStatusToolsDisplay::onInitialize()
{
	ROS_INFO("CobotStatusToolsDisplay::onInitialize()");
	md = this;
	Display::onInitialize();

	rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new CobotStatusToolsWidget( this, context_, window_context ? window_context->getParentWindow() : NULL);

  if (window_context)
  {
    frame_dock_ = window_context->addPane(getName(), frame_);
    connect(frame_dock_, SIGNAL(visibilityChanged(bool)), this, SLOT(cobotPanelVisibilityChange(bool)));
    frame_dock_->setIcon(getIcon());
  }
/*
	frame_->srv_teach_enable = nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_teach/enable");
	frame_->srv_jog_enable = nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_jog/enable");
	frame_->pub_jog = nh_.advertise<cobot_msgs::Jog>("/cobot/cobot_jog", 100);
*/
	frame_->srv_fk = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
	frame_->srv_read_point_file = nh_.serviceClient<cobot_msgs::ReadJointStateFile>("/cobot/cobot_core/read_js_file");
	frame_->srv_edit_js_file = nh_.serviceClient<cobot_msgs::EditJointStateFile>("/cobot/cobot_core/edit_js_file");
	frame_->srv_cobot_planning = nh_.serviceClient<cobot_planner::CobotPlanning>("/cobot/planning");
	frame_->updatePointsTable();
	sub_js = nh_.subscribe("/cobot/joint_states", 100, callback_js);
}

void CobotStatusToolsDisplay::callback_js(const sensor_msgs::JointState &_js) {
	md->js = _js;
	Q_EMIT md->jsUpdate();
}

void CobotStatusToolsDisplay::cobotPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}

void CobotStatusToolsDisplay::onEnable()
{
  Display::onEnable();
}

void CobotStatusToolsDisplay::onDisable()
{
  Display::onDisable();
}

void CobotStatusToolsDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
}

void CobotStatusToolsDisplay::load(const rviz::Config& config)
{
  Display::load(config);
}

void CobotStatusToolsDisplay::save(rviz::Config config) const
{
  Display::save(config);
}

void CobotStatusToolsDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
}

void CobotStatusToolsDisplay::reset()
{
	Display::reset();
}

void CobotStatusToolsDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  if (frame_dock_)
  {
    frame_dock_->setWindowTitle(name);
    frame_dock_->setObjectName(name);
  }
}
