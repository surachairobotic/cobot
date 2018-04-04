#include <MyDisplay.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/bool_property.h>

#include "ui_multimovedisplay.h"

using namespace my_plugin;

static const std::string& FRAME_ID = "/base_link";
static const std::string PLANNING_GROUP = "arm";

MyDisplay::MyDisplay() : nh_(), spinner(1)
		,irobot_start_state(0)
/*	,irobot_start_state( "robot_description", 
  										 "start_state_robot_pub", 
											 "start_state_robot_markers",
											 "start_state_robot_imarkers",
											 "arm",
											 "/base_link",
											 "robot_start_state")

	,irobot_goal_state( "robot_description", 
  										 "goal_state_robot_pub", 
											 "goal_state_robot_markers",
											 "goal_state_robot_imarkers",
											 "arm",
											 "/base_link",
											 "robot_start_state")
*/
{
	ROS_WARN("MyDisplay constructor");
	Display();
  frame_ = NULL;
	frame_dock_ = NULL;

	irobot_start_state = new InteractiveRobot( "robot_description", 
																						 "start_state_robot_pub", 
																						 "start_state_robot_markers",
																						 "start_state_robot_imarkers",
																						 "arm",
																						 "/base_link",
																						 "robot_start_state",
																						 this);
//	robot_start_ = nh_.subscribe<std_msgs::String>("my_topic", 1, callbackRobotStartState());
//	robot_start_ = nh_.subscribe("direction", 128, MyDisplay::callbackRobotStartState);
//	robot_start_ = nh_.subscribe("/start_state_robot_pub", 128, &MyDisplay::callbackRobotStartState, this);
//	robot_goal_ = nh_.subscribe("/goal_state_robot_pub", 128, &MyDisplay::callbackRobotGoalState, this);
//  ros::NodeHandle node_handle;
//  ros::AsyncSpinner spinner(1);
  spinner.start();

  irobot_start_state->setUserCallback(callbackRobotStartState);
//  irobot_goal_state.setUserCallback(callbackRobotGoalState);

	debug_message = "";
}

MyDisplay::~MyDisplay()
{
  if (frame_dock_)
    delete frame_dock_;
}

void MyDisplay::onInitialize()
{
	ROS_INFO("MyDisplay::onInitialize()");
	Display::onInitialize();

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new MyFrame( this, 
												context_, 
												window_context ? window_context->getParentWindow() : NULL);
//  resetStatusTextColor();
//  addStatusText("Initialized.");

  // immediately switch to next trajectory display after planning
//  connect(frame_, SIGNAL(planningFinished()), trajectory_visual_.get(), SLOT(interruptCurrentDisplay()));

  if (window_context)
  {
    frame_dock_ = window_context->addPane(getName(), frame_);
    connect(frame_dock_, SIGNAL(visibilityChanged(bool)), this, SLOT(motionPanelVisibilityChange(bool)));
    frame_dock_->setIcon(getIcon());
  }

  if (frame_)
  {
//    QString host;
    frame_->ui_->lineEdit_x->setText(QString("onInitialize"));
	}

}

void MyDisplay::motionPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}


void MyDisplay::callbackRobotStartState(InteractiveRobot &robot)
{
		ROS_WARN("callbackRobotStartState !!!");
//		robot.my_display->frame_->newLabel(robot.getPose());
		robot.my_display->frame_->ui_->lineEdit_x->setText(QString::fromStdString("xxxx"));
//		robot.getPose();

//		frame_->ui_->lineEdit_x->setText(QString::fromStdString(str.state.joint_state.name[0]));
//		QString tmp = "";
//		for(int i=0; i<robot_pose.size(); i++) {
//			tmp = tmp + QString::fromStdString("[");
//			tmp = tmp + QString::number(i);
//			tmp = tmp + QString::fromStdString("]");
//			tmp = tmp + QString::fromStdString(PoseString(robot_pose[i]));
//			tmp = tmp + QString::fromStdString("\r\n");
//		}

//			tmp = tmp + QString::fromStdString(str.state.joint_state.name[i]);
//			tmp = tmp + QString::fromStdString(", ");
//			tmp = tmp + QString::number(str.state.joint_state.position[i]);
//			tmp = tmp + QString::fromStdString(", ");
//			tmp = tmp + QString::number(str.state.joint_state.velocity[0]);
//			tmp = tmp + QString::fromStdString(", ");
//			tmp = tmp + QString::number(str.state.joint_state.effort[0]);

			
//		tmp = tmp + QString::fromStdString("\r\n");
//		frame_->ui_->plainTextEdit_info->setPlainText(frame_->ui_->plainTextEdit_info->toPlainText()
//																									+ tmp);	
//	}

}
/*
void MyDisplay::callbackRobotGoalState(InteractiveRobot &robot)
{
	ROS_WARN("callbackRobotGoalState !!!");
}
*/

std::string MyDisplay::double2string(double x)
{
	std::ostringstream o;
	if (!(o << x))
		o << "stringify(double)";
	return o.str();
}

void MyDisplay::onEnable()
{
  Display::onEnable();
}

void MyDisplay::onDisable()
{
  Display::onDisable();
}

void MyDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
}

void MyDisplay::load(const rviz::Config& config)
{
  Display::load(config);
}

void MyDisplay::save(rviz::Config config) const
{
  Display::save(config);
}

void MyDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
}

void MyDisplay::reset()
{
	Display::reset();
}

void MyDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  if (frame_dock_)
  {
    frame_dock_->setWindowTitle(name);
    frame_dock_->setObjectName(name);
  }
}

