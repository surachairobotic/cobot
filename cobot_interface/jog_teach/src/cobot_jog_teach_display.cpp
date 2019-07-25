#include <cobot_jog_teach_display.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/bool_property.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <QtMath>
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/Jog.h"
#include "moveit_msgs/GetPositionFK.h"
#include "cobot_planner/CobotPlanning.h"

#include "ui_cobot_jog_teach.h"

using namespace cobot_interface;

CobotJogTeachDisplay *md = NULL;

CobotJogTeachDisplay::CobotJogTeachDisplay() : nh_(), spinner(1)
{
	ROS_WARN("CobotJogTeachDisplay constructor");
	Display();
	frame_ = NULL;
	frame_dock_ = NULL;
	spinner.start();
}

CobotJogTeachDisplay::~CobotJogTeachDisplay()
{
	ROS_INFO("CobotJogTeachDisplay::~CobotJogTeachDisplay()");
	sub_js.shutdown();
	if (frame_dock_)
    delete frame_dock_;
}

void CobotJogTeachDisplay::onInitialize()
{
	ROS_INFO("CobotJogTeachDisplay::onInitialize()");
	md = this;
	Display::onInitialize();

	rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new CobotJogTeachWidget( this, context_, window_context ? window_context->getParentWindow() : NULL);

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
	sub_js = nh_.subscribe("/cobot/joint_states", 100, callback_js);
}

void CobotJogTeachDisplay::callback_js(const sensor_msgs::JointState &_js) {
	md->js = _js;
	Q_EMIT md->jsUpdate();
}

void CobotJogTeachDisplay::cobotPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}

void CobotJogTeachDisplay::onEnable()
{
  Display::onEnable();
}

void CobotJogTeachDisplay::onDisable()
{
  Display::onDisable();
}

void CobotJogTeachDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
}

void CobotJogTeachDisplay::load(const rviz::Config& config)
{
  Display::load(config);
}

void CobotJogTeachDisplay::save(rviz::Config config) const
{
  Display::save(config);
}

void CobotJogTeachDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
}

void CobotJogTeachDisplay::reset()
{
	Display::reset();
}

void CobotJogTeachDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  if (frame_dock_)
  {
    frame_dock_->setWindowTitle(name);
    frame_dock_->setObjectName(name);
  }
}
