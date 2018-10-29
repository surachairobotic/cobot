#include <MyFrame.h>
#include <MyDisplay.h>
#include <eigen_conversions/eigen_msg.h>
#include "cobot_kinematic.h"

#include "ui_multimovedisplay.h"

using namespace my_plugin;

void MyFrame::planButtonClicked()
{
  planning_display_->addBackgroundJob(boost::bind(&MyFrame::computePlanButtonClicked, this), "compute "
                                                                                                         "plan");
}

void MyFrame::computePlanButtonClicked()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->result_label_1->setText("Planning...");
  ui_->result_label_2->setText("Planning...");

  configureForPlanning();
	bool plan_is_ok = false;

	// Step A : current state --> start state
	moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
  move_group_->setStartState(*current_state);
	move_group_->setJointValueTarget(*start_state);
  current_plan_1.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
	if(move_group_->plan(*current_plan_1) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
		plan_is_ok = true;

    // Success
    ui_->result_label_1->setText(QString("Time 1: ").append(QString::number(current_plan_1->planning_time_, 'f', 3)));

	  dis_traj_1.trajectory_start = current_plan_1->start_state_;
		dis_traj_1.trajectory.clear();
		trajectory_1 = current_plan_1->trajectory_;
	  dis_traj_1.trajectory.push_back(trajectory_1);
		pub_plan_1.publish(dis_traj_1);
  }
  else
  {
    current_plan_1.reset();

    // Failure
    ui_->result_label_1->setText("Failed");
  }

	// Step B : start state --> goal state
  move_group_->setStartState(*start_state);
	move_group_->setJointValueTarget(*goal_state);
  current_plan_2.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
	if(move_group_->plan(*current_plan_2) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    // Success
    ui_->result_label_2->setText(QString("Time 2: ").append(QString::number(current_plan_2->planning_time_, 'f', 3)));

	  dis_traj_2.trajectory_start = current_plan_2->start_state_;
		dis_traj_2.trajectory.clear();
		trajectory_2 = current_plan_2->trajectory_;
	  dis_traj_2.trajectory.push_back(trajectory_2);
		pub_plan_2.publish(dis_traj_2);
  }
  else
  {
		plan_is_ok = false;
    current_plan_2.reset();

    // Failure
    ui_->result_label_2->setText("Failed");
  }

	if(plan_is_ok)
	{
		ui_->execute_button->setEnabled(true);
	  dis_traj_3.trajectory_start = dis_traj_1.trajectory_start;
		dis_traj_3.trajectory.clear();
	  dis_traj_3.trajectory.push_back(trajectory_1);
	  dis_traj_3.trajectory.push_back(trajectory_2);
		pub_plan_3.publish(dis_traj_3);
	}

  Q_EMIT planningFinished();
}

void MyFrame::configureForPlanning()
{
  move_group_->setPlanningTime(ui_->planning_time->value());
  move_group_->setNumPlanningAttempts(ui_->planning_attempts->value());
  move_group_->setMaxVelocityScalingFactor(ui_->velocity_scaling_factor->value());
  move_group_->setMaxAccelerationScalingFactor(ui_->acceleration_scaling_factor->value());

  configureWorkspace();
}

void MyFrame::configureWorkspace()
{
  robot_model::VariableBounds bx, by, bz;
  bx.position_bounded_ = by.position_bounded_ = bz.position_bounded_ = true;

  bx.min_position_ = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  bx.max_position_ = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  by.min_position_ = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  by.max_position_ = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  bz.min_position_ = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  bz.max_position_ = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;

  if (move_group_)
    move_group_->setWorkspace(bx.min_position_, by.min_position_, bz.min_position_, 
													  	bx.max_position_, by.max_position_, bz.max_position_);
}

void MyFrame::executeButtonClicked()
{
	ROS_INFO("executeButtonClicked() !!!");
  ui_->execute_button->setEnabled(false);
  // execution is done in a separate thread, to not block other background jobs by blocking for synchronous execution
  planning_display_->spawnBackgroundJob(boost::bind(&MyFrame::computeExecuteButtonClicked, this));
}

void MyFrame::computeExecuteButtonClicked()
{
	ROS_INFO("computeExecuteButtonClicked() !!!");
  if (move_group_ && current_plan_1 && current_plan_2)
  {
    ui_->stop_button->setEnabled(true);  // enable stopping

    bool success_1 = move_group_->execute(*current_plan_1) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    bool success_2 = move_group_->execute(*current_plan_2) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    onFinishedExecution(success_1 && success_2);
  }
}

void MyFrame::onFinishedExecution(bool success)
{
	ROS_INFO("onFinishedExecution() !!!");
  // visualize result of execution
  if (success) {
  	ui_->result_label_1->setText("Executed");
  	ui_->result_label_2->setText("Executed");
	}
/*		if (state)
    	ui_->result_label_1->setText("Executed");
		else
    	ui_->result_label_2->setText("Executed");
*/
  else {
  	ui_->result_label_1->setText(!ui_->stop_button->isEnabled() ? "Stopped" : "Failed");
  	ui_->result_label_2->setText(!ui_->stop_button->isEnabled() ? "Stopped" : "Failed");
}
/*		if (state)
    	ui_->result_label_1->setText(!ui_->stop_button->isEnabled() ? "Stopped" : "Failed");
		else
    	ui_->result_label_2->setText(!ui_->stop_button->isEnabled() ? "Stopped" : "Failed");
*/
  // disable stop button
  ui_->stop_button->setEnabled(false);

  // update query start state to current if neccessary
  //if (ui_->start_state_selection->currentText() == "<current>")
  //  useStartStateButtonClicked();
}

void MyFrame::stopButtonClicked()
{
  ui_->stop_button->setEnabled(false);  // avoid clicking again
  planning_display_->addBackgroundJob(boost::bind(&MyFrame::computeStopButtonClicked, this), "stop");
}

void MyFrame::computeStopButtonClicked()
{
  if (move_group_)
    move_group_->stop();
}

void MyFrame::upJ1Clicked()
{
	ROS_INFO("upJ1Clicked !!!");
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[0] += ui_->lineEdit_res->text().toDouble();
	ui_->lineEdit_j1->setText(QString::number(ik_seed_state[0], 'f', 4));
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::upJ2Clicked()
{
	ROS_INFO("upJ2Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[1] += ui_->lineEdit_res->text().toDouble();
	ui_->lineEdit_j2->setText(QString::number(ik_seed_state[1], 'f', 4));
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::upJ3Clicked()
{
	ROS_INFO("upJ3Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[2] += ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::upJ4Clicked()
{
	ROS_INFO("upJ4Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[3] += ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::upJ5Clicked()
{
	ROS_INFO("upJ5Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[4] += ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::upJ6Clicked()
{
	ROS_INFO("upJ6Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[5] += ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downJ1Clicked()
{
	ROS_INFO("downJ1Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[0] -= ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downJ2Clicked()
{
	ROS_INFO("downJ2Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[1] -= ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downJ3Clicked()
{
	ROS_INFO("downJ3Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[2] -= ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downJ4Clicked()
{
	ROS_INFO("downJ4Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[3] -= ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downJ5Clicked()
{
	ROS_INFO("downJ5Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[4] -= ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downJ6Clicked()
{
	ROS_INFO("downJ6Clicked !!!");	
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	ik_seed_state[5] -= ui_->lineEdit_res->text().toDouble();
	pubCurrentRobotState(ik_seed_state);
}

void MyFrame::downXClicked()
{
	ROS_INFO("downXClicked !!!");

	geometry_msgs::Pose pose = getEEFpose();
	pose.position.x -= ui_->lineEdit_res->text().toDouble();

	inPose(pose);
}

void MyFrame::downYClicked()
{
	ROS_INFO("downYClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();
	pose.position.y -= ui_->lineEdit_res->text().toDouble();

	inPose(pose);
}

void MyFrame::downZClicked()
{
	ROS_INFO("downZClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();
	pose.position.z -= ui_->lineEdit_res->text().toDouble();

	inPose(pose);
}

void MyFrame::downRXClicked()
{
	ROS_INFO("downRXClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	roll -= ui_->lineEdit_res->text().toDouble();
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	inPose(pose);
}

void MyFrame::downRYClicked()
{
	ROS_INFO("downRYClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	pitch -= ui_->lineEdit_res->text().toDouble();
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	inPose(pose);
}

void MyFrame::downRZClicked()
{
	ROS_INFO("downRZClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw -= ui_->lineEdit_res->text().toDouble();
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	inPose(pose);
}

void MyFrame::upXClicked()
{
	ROS_INFO("upXClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();
	pose.position.x += ui_->lineEdit_res->text().toDouble();

	inPose(pose);
}

void MyFrame::upYClicked()
{
	ROS_INFO("upYClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();
	pose.position.y += ui_->lineEdit_res->text().toDouble();

	inPose(pose);
}

void MyFrame::upZClicked()
{
	ROS_INFO("upZClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();
	pose.position.z += ui_->lineEdit_res->text().toDouble();

	inPose(pose);
}

void MyFrame::upRXClicked()
{
	ROS_INFO("upRXClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	roll += ui_->lineEdit_res->text().toDouble();
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	inPose(pose);
}

void MyFrame::upRYClicked()
{
	ROS_INFO("upRYClicked !!!");	

	geometry_msgs::Pose pose = getEEFpose();

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	pitch += ui_->lineEdit_res->text().toDouble();
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	inPose(pose);
}

void MyFrame::upRZClicked()
{
	ROS_INFO("upRZClicked !!!");

	geometry_msgs::Pose pose = getEEFpose();

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw += ui_->lineEdit_res->text().toDouble();
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	inPose(pose);
}

void MyFrame::pushButtonClicked()
{
	ROS_INFO("pushButtonClicked !!!");
	std::vector<double> home{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	pubCurrentRobotState(home);
}

void MyFrame::pushButton_2Clicked()
{
	ROS_INFO("pushButton_2Clicked !!!");
/*	iSol = ((iSol+1) % solutions.size());
	sensor_msgs::JointState msgJoint;
	msgJoint.name = planning_display_->group_->getActiveJointModelNames();
	msgJoint.position = solutions[iSol];
	ROS_INFO("iSol = %d from %d", (int)iSol, (int)solutions.size());

  joint_debug_publisher_.publish(msgJoint);
*/

/*
	std::vector<geometry_msgs::Pose> poseJoint;
	std::vector<double> theta{0, 0, 0, 0, 0, 0};
	poseJoint = coco.computeFK(theta);
	for(int i=0; i<poseJoint.size(); i++)
		ROS_INFO("poseJoint[%d] : %lf, %lf, %lf", i, poseJoint[i].position.x, poseJoint[i].position.y, poseJoint[i].position.z);
*/

	std::vector<double> theta{0, 0, 0, 0, 0, 0};
	theta[0] = ui_->input_j1->text().toDouble();
	theta[1] = ui_->input_j2->text().toDouble();
	theta[2] = ui_->input_j3->text().toDouble();
	theta[3] = ui_->input_j4->text().toDouble();
	theta[4] = ui_->input_j5->text().toDouble();
	theta[5] = ui_->input_j6->text().toDouble();

	std::vector<double> offset{0.0, -0.016406, -0.009949, 0.028798, -0.000698, 0.0};
	if(ui_->checkBox->checkState() == 2)
	{
		if(theta.size() == offset.size())
		{
			ROS_INFO("In offset !!!");
			for(int i=0; i<theta.size(); i++)
				theta[i] -= offset[i];
		}
	}
	pubCurrentRobotState(theta);

/*
	pubCurrentRobotState(test_point[index]);
	index = (index+1) % 4;
*/
}

void MyFrame::btn_inputClicked()
{
	geometry_msgs::Pose pose;
	pose.position.x = ui_->input_eef_x->text().toDouble();
	pose.position.y = ui_->input_eef_y->text().toDouble();
	pose.position.z = ui_->input_eef_z->text().toDouble();

	double roll, pitch, yaw;
	roll = ui_->input_eef_rx->text().toDouble();
	pitch = ui_->input_eef_ry->text().toDouble();
	yaw = ui_->input_eef_rz->text().toDouble();

	ROS_INFO("%lf, %lf, %lf | %lf, %lf, %lf", pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);

	tf::Quaternion q_ori;
	q_ori.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q_ori, pose.orientation);

	

	inPose(pose);
}
