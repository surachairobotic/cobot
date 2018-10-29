#include <MyDisplay.h>
#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/bool_property.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <QtMath>

#include "ui_multimovedisplay.h"

using namespace my_plugin;

static const std::string& FRAME_ID = "/base_link";
static const std::string PLANNING_GROUP = "arm";

MyDisplay::MyDisplay() : nh_(), spinner(1),
	irobot_start_state(0),
	irobot_goal_state(0),
	rm_loader_(robot_description),

	// create publishers for markers and robot state
	robot_state_publisher_(nh_.advertise<moveit_msgs::DisplayRobotState>("current_state_robot_pub", 1))
{
	ROS_WARN("MyDisplay constructor");
	Display();
  frame_ = NULL;
	frame_dock_ = NULL;

  // get the RobotModel loaded from urdf and srdf files
  robot_model_ = rm_loader_.getModel();
  if (!robot_model_)
    ROS_ERROR("Could not load robot description");

/*	for(int i=0; i<robot_model_->getJointModelCount(); i++)
	{
		const robot_state::JointModel *j = robot_model_->getJointModel(i);
		const std::string name = j->getName();
		const robot_model::JointModel::JointType type = j->getType();
		if( name.size()==0 || 
				(name[0]!='j' && name[0]!='J') || 
				type!=robot_model::JointModel::REVOLUTE )
			continue;
		sJointsName.push_back(name);
	}
*/
  // create a RobotState to keep track of the current robot pose
  robotCurrentState.reset(new robot_state::RobotState(robot_model_));
  if (!robotCurrentState)
    ROS_ERROR("Could not get RobotState from Model");
  robotCurrentState->setToDefaultValues();
	
  // Prepare to move the "right_arm" group
  group_ = robotCurrentState->getJointModelGroup(planning_group);
	sJointsName = group_->getActiveJointModelNames();
  end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robotCurrentState->getGlobalLinkTransform(end_link);

  // Load solver
	solver = group_->getSolverInstance();

  if (!solver)
  {
    ROS_ERROR("No kinematics solver instantiated for group '%s'", group_->getName().c_str());
    while(1);
  }

	irobot_start_state = new InteractiveRobot( "robot_description", 
																						 "start_state_robot_pub", 
																						 "start_state_robot_markers",
																						 "start_state_robot_imarkers",
																						 "arm",
																						 "/base_link",
																						 "robot_start_state",
																						 this);
	irobot_start_state->setColor(irobot_start_state->createRandColor());
	irobot_goal_state = new InteractiveRobot( "robot_description", 
																						 "goal_state_robot_pub", 
																						 "goal_state_robot_markers",
																						 "goal_state_robot_imarkers",
																						 "arm",
																						 "/base_link",
																						 "robot_goal_state",
																						 this);
	irobot_goal_state->setColor(irobot_goal_state->createRandColor());

  spinner.start();
	debug_message = "";
}

MyDisplay::~MyDisplay()
{
	ROS_INFO("MyDisplay::~MyDisplay()");
//	joint_state_subscriber_.shutdown();
/*	while(in_callback){
		ROS_INFO("in_callback . . .");
	}
*/
  if (frame_dock_)
    delete frame_dock_;
}

MyDisplay *md = NULL;
void MyDisplay::onInitialize()
{
	md = this;
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
    frame_->ui_->lineEdit_x->setText(QString("onInitialize"));
	  irobot_start_state->setUserCallback(callbackRobotStartState);
		irobot_goal_state->setUserCallback(callbackRobotGoalState);
		updateCurrentRobotState();

/*
		connect(frame_->ui_->btn_x_inc, SIGNAL(clicked()), this, SLOT(btn_x_inc_Clicked()));
		connect(frame_->ui_->btn_x_dec, SIGNAL(clicked()), this, SLOT(btn_x_dec_Clicked()));
		connect(frame_->ui_->btn_y_inc, SIGNAL(clicked()), this, SLOT(btn_y_inc_Clicked()));
		connect(frame_->ui_->btn_y_dec, SIGNAL(clicked()), this, SLOT(btn_y_dec_Clicked()));
		connect(frame_->ui_->btn_z_inc, SIGNAL(clicked()), this, SLOT(btn_z_inc_Clicked()));
		connect(frame_->ui_->btn_z_dec, SIGNAL(clicked()), this, SLOT(btn_z_dec_Clicked()));
		connect(frame_->ui_->btn_rx_inc, SIGNAL(clicked()), this, SLOT(btn_rx_inc_Clicked()));
		connect(frame_->ui_->btn_rx_dec, SIGNAL(clicked()), this, SLOT(btn_rx_dec_Clicked()));
		connect(frame_->ui_->btn_ry_inc, SIGNAL(clicked()), this, SLOT(btn_ry_inc_Clicked()));
		connect(frame_->ui_->btn_ry_dec, SIGNAL(clicked()), this, SLOT(btn_ry_dec_Clicked()));
		connect(frame_->ui_->btn_rz_inc, SIGNAL(clicked()), this, SLOT(btn_rz_inc_Clicked()));
		connect(frame_->ui_->btn_rz_dec, SIGNAL(clicked()), this, SLOT(btn_rz_dec_Clicked()));
*/

		updateUIJointsLabel();
	}

	joint_state_subscriber_ = nh_.subscribe("/joint_states", 1, callbackJointState);
}

/*
void MyDisplay::btn_x_inc_Clicked()
{
	frame_->robot_pose[5].position.x += 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_x_dec_Clicked()
{
	frame_->robot_pose[5].position.x -= 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_y_inc_Clicked()
{
	frame_->robot_pose[5].position.y += 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_y_dec_Clicked()
{
	frame_->robot_pose[5].position.y -= 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_z_inc_Clicked()
{
	frame_->robot_pose[5].position.z += 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_z_dec_Clicked()
{
	frame_->robot_pose[5].position.z -= 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_rx_inc_Clicked()
{
	frame_->robot_pose[5].orientation.x += 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_rx_dec_Clicked()
{
	frame_->robot_pose[5].orientation.x -= 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_ry_inc_Clicked()
{
	frame_->robot_pose[5].orientation.y += 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_ry_dec_Clicked()
{
	frame_->robot_pose[5].orientation.y -= 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_rz_inc_Clicked()
{
	frame_->robot_pose[5].orientation.z += 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
void MyDisplay::btn_rz_dec_Clicked()
{
	frame_->robot_pose[5].orientation.z -= 0.02;
	irobot_start_state->setGroupPose(frame_->robot_pose[5]);
}
*/
void MyDisplay::motionPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}

void MyDisplay::callbackRobotStartState(InteractiveRobot &robot)
{
		ROS_WARN("callbackRobotStartState !!!");
		std::vector<geometry_msgs::Pose> vPose = robot.getPose();
		robot.my_display->frame_->robot_start_pose = vPose;
		robot.my_display->frame_->start_state = robot.robotState();
		//robot.my_display->robotCurrentState = robot.robotState();

		int joint = 5;
		robot.my_display->frame_->ui_->lineEdit_x->setText(QString::number(vPose[joint].position.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_y->setText(QString::number(vPose[joint].position.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_z->setText(QString::number(vPose[joint].position.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_rx->setText(QString::number(vPose[joint].orientation.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_ry->setText(QString::number(vPose[joint].orientation.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_rz->setText(QString::number(vPose[joint].orientation.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_rw->setText(QString::number(vPose[joint].orientation.w, 'f', 4));

		joint = 4;
		robot.my_display->frame_->ui_->lineEdit_j1_x->setText(QString::number(vPose[joint].position.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j1_y->setText(QString::number(vPose[joint].position.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j1_z->setText(QString::number(vPose[joint].position.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j1_rx->setText(QString::number(vPose[joint].orientation.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j1_ry->setText(QString::number(vPose[joint].orientation.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j1_rz->setText(QString::number(vPose[joint].orientation.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j1_rw->setText(QString::number(vPose[joint].orientation.w, 'f', 4));

		joint = 3;
		robot.my_display->frame_->ui_->lineEdit_j2_x->setText(QString::number(vPose[joint].position.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j2_y->setText(QString::number(vPose[joint].position.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j2_z->setText(QString::number(vPose[joint].position.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j2_rx->setText(QString::number(vPose[joint].orientation.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j2_ry->setText(QString::number(vPose[joint].orientation.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j2_rz->setText(QString::number(vPose[joint].orientation.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j2_rw->setText(QString::number(vPose[joint].orientation.w, 'f', 4));

		joint = 2;
		robot.my_display->frame_->ui_->lineEdit_j3_x->setText(QString::number(vPose[joint].position.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j3_y->setText(QString::number(vPose[joint].position.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j3_z->setText(QString::number(vPose[joint].position.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j3_rx->setText(QString::number(vPose[joint].orientation.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j3_ry->setText(QString::number(vPose[joint].orientation.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j3_rz->setText(QString::number(vPose[joint].orientation.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j3_rw->setText(QString::number(vPose[joint].orientation.w, 'f', 4));

		joint = 1;
		robot.my_display->frame_->ui_->lineEdit_j4_x->setText(QString::number(vPose[joint].position.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j4_y->setText(QString::number(vPose[joint].position.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j4_z->setText(QString::number(vPose[joint].position.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j4_rx->setText(QString::number(vPose[joint].orientation.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j4_ry->setText(QString::number(vPose[joint].orientation.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j4_rz->setText(QString::number(vPose[joint].orientation.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j4_rw->setText(QString::number(vPose[joint].orientation.w, 'f', 4));

		joint = 0;
		robot.my_display->frame_->ui_->lineEdit_j5_x->setText(QString::number(vPose[joint].position.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j5_y->setText(QString::number(vPose[joint].position.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j5_z->setText(QString::number(vPose[joint].position.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j5_rx->setText(QString::number(vPose[joint].orientation.x, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j5_ry->setText(QString::number(vPose[joint].orientation.y, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j5_rz->setText(QString::number(vPose[joint].orientation.z, 'f', 4));
		robot.my_display->frame_->ui_->lineEdit_j5_rw->setText(QString::number(vPose[joint].orientation.w, 'f', 4));
}

void MyDisplay::callbackRobotGoalState(InteractiveRobot &robot)
{
	ROS_WARN("callbackRobotGoalState !!!");
	geometry_msgs::Pose pose = robot.getBackPose();
	robot.my_display->frame_->robot_goal_pose = pose;
	robot.my_display->frame_->goal_state = robot.robotState();
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

void MyDisplay::callbackJointState(const sensor_msgs::JointState& msgJoint)
{
//	in_callback = true;
	md->jointCurrentState = msgJoint;
	md->updateCurrentRobotState();
//	if(md->frame_)
//		md->frame_->pubUiSeed(md->jointCurrentState.position);
	md->updateUIJointsPosition();
//	in_callback = false;
}

void MyDisplay::updateCurrentRobotState()
{
	moveit_msgs::DisplayRobotState msg;
	robot_state::RobotStatePtr current;
	current = frame_->move_group_->getCurrentState();
	if(current != NULL)
	{
		robotCurrentState = current;
	}
	else
	{
		ROS_WARN("no current state");
		return;
	}
	robot_state::robotStateToRobotStateMsg(*robotCurrentState, msg.state);
	robot_state_publisher_.publish(msg);
}

void MyDisplay::updateUIJointsPosition()
{
	frame_->pubUiSeed(jointCurrentState.position);
/*
	ROS_INFO("updateUIJointsPosition !!!");

	for(int i=0; i<6; i++) {
		if(jointCurrentState.position[i] < -M_PI || jointCurrentState.position[i] > M_PI)
			ROS_INFO("jointCurrentState.position[%d] = %f", i, jointCurrentState.position[i]);
	}
*/
/*
	double angle = qRadiansToDegrees(jointCurrentState.position[0]);
	frame_->ui_->lineEdit_j1->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(jointCurrentState.position[1]);
	frame_->ui_->lineEdit_j2->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(jointCurrentState.position[2]);
	frame_->ui_->lineEdit_j3->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(jointCurrentState.position[3]);
	frame_->ui_->lineEdit_j4->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(jointCurrentState.position[4]);
	frame_->ui_->lineEdit_j5->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(jointCurrentState.position[5]);
	frame_->ui_->lineEdit_j6->setText(QString::number(angle, 'f', 4));
*/
/*
	geometry_msgs::Pose pose = irobot_goal_state->getBackPose();
	frame_->ui_->lineEdit_eef_x->setText(QString::number(pose.position.x, 'f', 4));
	frame_->ui_->lineEdit_eef_y->setText(QString::number(pose.position.y, 'f', 4));
	frame_->ui_->lineEdit_eef_z->setText(QString::number(pose.position.z, 'f', 4));
	angle = qRadiansToDegrees(pose.orientation.x);
	frame_->ui_->lineEdit_eef_rx->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(pose.orientation.y);
	frame_->ui_->lineEdit_eef_ry->setText(QString::number(angle, 'f', 4));
	angle = qRadiansToDegrees(pose.orientation.z);
	frame_->ui_->lineEdit_eef_rz->setText(QString::number(angle, 'f', 4));
*/
}

void MyDisplay::updateUIJointsLabel()
{
	ROS_INFO("updateUIJointsLabel !!!");
//	sJointsName = robotCurrentState->getVariableNames();
	frame_->ui_->label_j1->setText(QString::fromStdString(sJointsName[0]));
	frame_->ui_->label_j2->setText(QString::fromStdString(sJointsName[1]));
	frame_->ui_->label_j3->setText(QString::fromStdString(sJointsName[2]));
	frame_->ui_->label_j4->setText(QString::fromStdString(sJointsName[3]));
	frame_->ui_->label_j5->setText(QString::fromStdString(sJointsName[4]));
	frame_->ui_->label_j6->setText(QString::fromStdString(sJointsName[5]));
	ROS_INFO("updateUIJointsLabel - END !!!");
}

void MyDisplay::addBackgroundJob(const boost::function<void()>& job, const std::string& name)
{
  background_process_.addJob(job, name);
}

void MyDisplay::spawnBackgroundJob(const boost::function<void()>& job)
{
  boost::thread t(job);
}

void MyDisplay::addMainLoopJob(const boost::function<void()>& job)
{
  boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void MyDisplay::waitForAllMainLoopJobs()
{
  boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
  while (!main_loop_jobs_.empty())
    main_loop_jobs_empty_condition_.wait(ulock);
}

void MyDisplay::clearJobs()
{
  background_process_.clear();
  {
    boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
    main_loop_jobs_.clear();
  }
}

