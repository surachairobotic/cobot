#include <cobot_interface_beta_display.h>
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
#include "cobot_msgs/PickPlacePoseArray.h"
#include "moveit_msgs/GetPositionFK.h"
#include "cobot_planner/CobotPlanning.h"
#include "std_msgs/Bool.h"

#include "ui_cobot_interface_beta.h"

using namespace cobot_interface;

CobotInterfaceBetaDisplay *md = NULL;

CobotInterfaceBetaDisplay::CobotInterfaceBetaDisplay() : nh_(), spinner(1)
{
	ROS_WARN("CobotInterfaceBetaDisplay constructor");
	Display();
	frame_ = NULL;
	frame_dock_ = NULL;
	spinner.start();
}

CobotInterfaceBetaDisplay::~CobotInterfaceBetaDisplay()
{
	ROS_INFO("CobotInterfaceBetaDisplay::~CobotInterfaceBetaDisplay()");
	sub_js.shutdown();
	if (frame_dock_)
    delete frame_dock_;
}

void CobotInterfaceBetaDisplay::onInitialize()
{
	ROS_INFO("CobotInterfaceBetaDisplay::onInitialize()");
	md = this;
	Display::onInitialize();

	rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new CobotInterfaceBetaWidget( this, context_, window_context ? window_context->getParentWindow() : NULL);

  if (window_context)
  {
    frame_dock_ = window_context->addPane(getName(), frame_);
    connect(frame_dock_, SIGNAL(visibilityChanged(bool)), this, SLOT(cobotPanelVisibilityChange(bool)));
    frame_dock_->setIcon(getIcon());
  }

	frame_->pub_jog = nh_.advertise<cobot_msgs::Jog>("/cobot/cobot_jog", 100);
	frame_->pub_pick_en = nh_.advertise<std_msgs::Bool>("/cobot/image2pose/enable", 10);
	frame_->pub_pick_cmd = nh_.advertise<std_msgs::String>("/cobot/image2pose/cmd", 10);
	// frame_->srv_teach_enable = nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_teach/enable");
	// frame_->srv_jog_enable = nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_jog/enable");
	// frame_->srv_fk = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
	// frame_->srv_edit_js_file = nh_.serviceClient<cobot_msgs::EditJointStateFile>("/cobot/cobot_core/edit_js_file");
	// frame_->srv_read_point_file = nh_.serviceClient<cobot_msgs::ReadJointStateFile>("/cobot/cobot_core/read_js_file");
	// frame_->srv_cobot_planning = nh_.serviceClient<cobot_planner::CobotPlanning>("/cobot/planning");

	sub_js = nh_.subscribe("/cobot/joint_states", 100, callback_js);
	sub_pppa = nh_.subscribe("cobot/image2pose/pickplace_array", 100, callback_pppa);

	pub_vacuum = nh_.advertise<std_msgs::Bool>("/cobot/message", 10);
	frame_->updatePointsTable();
}

void CobotInterfaceBetaDisplay::callback_js(const sensor_msgs::JointState &_js) {
	md->js = _js;
	Q_EMIT md->jsUpdate();
}

void CobotInterfaceBetaDisplay::callback_pppa(const cobot_msgs::PickPlacePoseArray &_msg) {
	md->pppa = _msg;
	Q_EMIT md->pppaUpdate();
}

void CobotInterfaceBetaDisplay::cobotPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}

void CobotInterfaceBetaDisplay::onEnable()
{
  Display::onEnable();
}

void CobotInterfaceBetaDisplay::onDisable()
{
  Display::onDisable();
}

void CobotInterfaceBetaDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
}

void CobotInterfaceBetaDisplay::load(const rviz::Config& config)
{
  Display::load(config);
}

void CobotInterfaceBetaDisplay::save(rviz::Config config) const
{
  Display::save(config);
}

void CobotInterfaceBetaDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
}

void CobotInterfaceBetaDisplay::reset()
{
	Display::reset();
}

void CobotInterfaceBetaDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  if (frame_dock_)
  {
    frame_dock_->setWindowTitle(name);
    frame_dock_->setObjectName(name);
  }
}
