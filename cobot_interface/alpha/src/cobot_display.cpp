#include <cobot_display.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/bool_property.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <QtMath>
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "cobot_msgs/ReadJointStateFile.h"

#include "ui_cobot_interface_alpha.h"

using namespace cobot_interface;

CobotDisplay::CobotDisplay() : nh_(), spinner(1)
{
	ROS_WARN("CobotDisplay constructor");
	Display();
	frame_ = NULL;
	frame_dock_ = NULL;
	spinner.start();
}

CobotDisplay::~CobotDisplay()
{
	ROS_INFO("CobotDisplay::~CobotDisplay()");
	if (frame_dock_)
    delete frame_dock_;
}

void CobotDisplay::onInitialize()
{
	ROS_INFO("CobotDisplay::onInitialize()");
	Display::onInitialize();
	rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new CobotWidget( this, context_, window_context ? window_context->getParentWindow() : NULL);

  if (window_context)
  {
    frame_dock_ = window_context->addPane(getName(), frame_);
    connect(frame_dock_, SIGNAL(visibilityChanged(bool)), this, SLOT(cobotPanelVisibilityChange(bool)));
    frame_dock_->setIcon(getIcon());
  }

	frame_->srv_teach_enable = nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_teach/enable");
	frame_->srv_read_point_file = nh_.serviceClient<cobot_msgs::ReadJointStateFile>("/cobot/cobot_core/read_js_file");
	frame_->srv_edit_js_file = nh_.serviceClient<cobot_msgs::EditJointStateFile>("/cobot/cobot_core/edit_js_file");
}

void CobotDisplay::cobotPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}

void CobotDisplay::onEnable()
{
  Display::onEnable();
}

void CobotDisplay::onDisable()
{
  Display::onDisable();
}

void CobotDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
}

void CobotDisplay::load(const rviz::Config& config)
{
  Display::load(config);
}

void CobotDisplay::save(rviz::Config config) const
{
  Display::save(config);
}

void CobotDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
}

void CobotDisplay::reset()
{
	Display::reset();
}

void CobotDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  if (frame_dock_)
  {
    frame_dock_->setWindowTitle(name);
    frame_dock_->setObjectName(name);
  }
}
