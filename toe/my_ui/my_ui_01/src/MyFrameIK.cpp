#include <MyFrame.h>
#include <MyDisplay.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

#include "ui_multimovedisplay.h"

using namespace my_plugin;

geometry_msgs::Pose MyFrame::getEEFpose(const robot_state::RobotStatePtr& robot)
{
  Eigen::Affine3d aff_pose = robot->getGlobalLinkTransform(planning_display_->end_link);
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(aff_pose, pose);

	return pose;
}

void MyFrame::inPose(geometry_msgs::Pose pose)
{
	ROS_INFO("MyFrame::inPose !!!");

/*	tf::Quaternion q_ori;
	double r=0, p=0, y=0;
	quaternionMsgToTF(pose.orientation , q_ori);
	q_ori.normalize();
	quaternionTFToMsg(q_ori, pose.orientation);
*/

	double norm = sqrt(pow(pose.orientation.x, 2) + pow(pose.orientation.y, 2) + pow(pose.orientation.z, 2) + pow(pose.orientation.w, 2));

	ROS_INFO("pose in [%lf %lf %lf, %lf %lf %lf %lf] | %lf", pose.position.x, pose.position.y, pose.position.z
																						, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, norm);
/*
	pose.position.x = 0.25;
	pose.position.y = 0.00;
	pose.position.z = 0.29503;
	pose.orientation.x = 1.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 0.0;
*/
	pub_jog_robot_state(pose);

//	tf::poseEigenToMsg(aff_pose, pose);
//	ui_->lineEdit_eef_x->setText(QString::number(pose.position.x, 'f', 4));
}

void MyFrame::pub_jog_robot_state(geometry_msgs::Pose pose)
{
	ROS_INFO("pub_jog_robot_state !!!");
//	joint_model_group = planning_display_->robotCurrentState->getJointModelGroup("arm");
	std::vector<geometry_msgs::Pose> ik_poses;
	ik_poses.push_back(pose);
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	for(int i=0; i<ik_seed_state.size(); i++)
		ROS_INFO("seed before[%d] : %lf", i, ik_seed_state[i]);
/*
	for(int i=0; i<ik_seed_state.size(); i++) {
		if(ik_seed_state[i] < 0)
			ik_seed_state[i] += M_PI;
		if(ik_seed_state[i] < 0)
			ik_seed_state[i] += M_PI;

		if(ik_seed_state[i] < -M_PI || ik_seed_state[i] > M_PI)
			ROS_WARN("seed over PI [%d] : %lf", i, ik_seed_state[i]);
		ROS_INFO("seed before[%d] : %lf", i, ik_seed_state[i]);
	}
*/
//	group_->getVariableDefaultPositions(ik_seed_state);
//	kinematics::KinematicsResult kinematic_result;
	moveit_msgs::MoveItErrorCodes error_code;	
	kinematics::KinematicsQueryOptions options; // = kinematics::KinematicsQueryOptions();
	options.return_approximate_solution = true;
	options.discretization_method = kinematics::DiscretizationMethods::NO_DISCRETIZATION;
	
	solutions.clear();

/*	std::vector<geometry_msgs::Pose> pose_fk;
	std::vector<std::string> link_names;
	link_names.push_back(planning_display_->end_link);
	if(solver->getPositionFK(link_names, 
													 ik_seed_state, 
													 pose_fk))
*/
/*	if(planning_display_->solver->getPositionIK(ik_poses, ik_seed_state, solutions, kinematic_result, options))
*/
//  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

	for(int i=0; i<ik_seed_state.size(); i++)
		consistency_limits.push_back(1.04719755); // 60 degree

	if(planning_display_->solver->searchPositionIK(pose, ik_seed_state, 1.0, consistency_limits, solution, error_code, options))
	{
		ROS_WARN("searchPositionIK : true");
		ROS_INFO("pose A [%lf, %lf, %lf | %lf, %lf, %lf, %lf]", pose.position.x, pose.position.y, pose.position.z
																													, pose.orientation.x, pose.orientation.y
																													, pose.orientation.z, pose.orientation.w);
		ROS_INFO("A : %lf, %lf, %lf, %lf, %lf, %lf", solution[0], solution[1], solution[2], solution[3], solution[4], solution[5]);

		std::vector<std::vector<double>> new_solutions;
		new_solutions.clear();
		new_solutions = coco.computeIK(pose);
		ROS_INFO("pose B [%lf, %lf, %lf | %lf, %lf, %lf, %lf]", pose.position.x, pose.position.y, pose.position.z
																													, pose.orientation.x, pose.orientation.y
																													, pose.orientation.z, pose.orientation.w);
		for(int i=0; i<new_solutions.size(); i++) {		
			ROS_INFO("B : %lf, %lf, %lf, %lf, %lf, %lf", new_solutions[i][0], new_solutions[i][1], new_solutions[i][2], new_solutions[i][3], new_solutions[i][4], new_solutions[i][5]);
		}

		//best_solution(ik_seed_state, solutions);		
	}
	else
		ROS_WARN("searchPositionIK : false");

	//offset : 0.000002 0.000000 -0.016406 -0.009949 0.028798 -0.000698 -0.076794
	std::vector<double> offset{0.0, -0.016406, -0.009949, 0.028798, -0.000698, 0.0};
	if(ui_->checkBox->checkState() == 2)
		if(solution.size() == offset.size())
		{
			ROS_INFO("In offset !!!");
			for(int i=0; i<solution.size(); i++)
				solution[i] -= offset[i];
		}
	pubCurrentRobotState(solution);
}

void MyFrame::best_solution(std::vector<double>& ik_seed_state, std::vector<std::vector<double>>& solutions)
{
	ROS_INFO("best_solution !!!");

	// sort solutions by their distance to the seed
//	joint_model_group = planning_display_->group_;
//	std::vector<double> ik_seed_state;
//	planning_display_->robotCurrentState->copyJointGroupPositions("arm", ik_seed_state);
//	group_->getVariableDefaultPositions(ik_seed_state);
	double dist;
	int sol_select = 0;
	bool first = true;
	for (int i = 0; i < solutions.size(); ++i)
	{
		double dist_from_seed = 0.0;
		for (int j = 0; j < ik_seed_state.size(); ++j)
		{
			dist_from_seed += fabs(fabs(ik_seed_state[j]) - fabs(solutions[i][j]));
		}

		if(first)
		{
			dist = dist_from_seed;
			sol_select = 0;
			first = false;
		}
		else if(dist_from_seed < dist)
		{
			dist = dist_from_seed;
			sol_select = i;
		}
//		dist.push_back(dist_from_seed);
//		solutions_obey_limits.push_back({ solutions[i], dist_from_seed });
	}
//	std::sort(solutions_obey_limits.begin(), solutions_obey_limits.end());

	double norm;
	ROS_INFO("solutions.size() = %d", (int)solutions.size());
	for(int j=0; j<solutions.size(); j++)
	{
		for(int i=0; i<solutions[sol_select].size(); i++)
			ROS_INFO("solution[%d] : %lf", i, solutions[j][i]);

		std::vector<geometry_msgs::Pose> pose_fk;
		std::vector<std::string> link_names;
		link_names.push_back(planning_display_->end_link);
		if(planning_display_->solver->getPositionFK(link_names, 
														 solutions[j], 
														 pose_fk)) {
			ROS_INFO("[%d] : FK true !!!", j);
			ROS_INFO("pose_fk.size : %d", (int)pose_fk.size());
			ROS_INFO("%lf, %lf, %lf | %lf, %lf, %lf, %lf", pose_fk[0].position.x
																										, pose_fk[0].position.y
																										, pose_fk[0].position.z
																										, pose_fk[0].orientation.x
																										, pose_fk[0].orientation.y
																										, pose_fk[0].orientation.z
																										, pose_fk[0].orientation.w);
		}
		else
			ROS_INFO("[%d] : FK false !!!", j);
	}

	sensor_msgs::JointState msgJoint;

	msgJoint.name = planning_display_->group_->getActiveJointModelNames();
	msgJoint.position = solutions[sol_select];
	ROS_INFO("sol_select = %d from %d", (int)sol_select, (int)solutions.size());


	ROS_INFO("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
	sol_select = 0;
	first = true;
/*	for(int i=0; i<solutions.size(); i++)
		for(int j=0; j<solutions[i].size(); j++)
		{
			if(solutions[i][j] < 0)
				solutions[i][j] += (M_PI*2);
//			if(solutions[i][j] < 0)
//				solutions[i][j] += M_PI;
		}
*/
	for (int i = 0; i < solutions.size(); ++i)
	{
		double dist_from_seed = 0.0;
		for (int j = 0; j < ik_seed_state.size(); ++j)
		{
			double d = ik_seed_state[j] - solutions[i][j];
			if(d <= (M_PI*2.0))	d += (M_PI*2.0);
			else if(d >= (M_PI*2.0))	d -= (M_PI*2.0);
			dist_from_seed += pow(d, 2);
		}

		if(first)
		{
			dist = dist_from_seed;
			sol_select = 0;
			first = false;
		}
		else if(dist_from_seed < dist)
		{
			dist = dist_from_seed;
			sol_select = i;
		}
	}

	ROS_INFO("solutions.size() = %d", (int)solutions.size());
	for(int j=0; j<solutions.size(); j++)
	{
		for(int i=0; i<solutions[sol_select].size(); i++)
			ROS_INFO("solution[%d] : %lf", i, solutions[j][i]);

		std::vector<geometry_msgs::Pose> pose_fk;
		std::vector<std::string> link_names;
		link_names.push_back(planning_display_->end_link);
		if(planning_display_->solver->getPositionFK(link_names, 
														 solutions[j], 
														 pose_fk)) {
			ROS_INFO("[%d] : FK true !!!", j);
			ROS_INFO("pose_fk.size : %d", (int)pose_fk.size());
			ROS_INFO("%lf, %lf, %lf | %lf, %lf, %lf, %lf", pose_fk[0].position.x
																										, pose_fk[0].position.y
																										, pose_fk[0].position.z
																										, pose_fk[0].orientation.x
																										, pose_fk[0].orientation.y
																										, pose_fk[0].orientation.z
																										, pose_fk[0].orientation.w);
		}
		else
			ROS_INFO("[%d] : FK false !!!", j);
	}

	msgJoint.name = planning_display_->group_->getActiveJointModelNames();
	msgJoint.position = solutions[sol_select];
	ROS_INFO("sol_select B = %d from %d", (int)sol_select, (int)solutions.size());

	//planning_display_->robotCurrentState->setVariablePositions(solutions[sol_select]);

/*	for(int i=0; i<msgJoint.name.size(); i++)
		ROS_INFO("name %d : %s", i, msgJoint.name[i].c_str());
	for(int i=0; i<msgJoint.position.size(); i++)
		ROS_INFO("position %d : %lf", i, msgJoint.position[i]);
*/

//  joint_state_pub.publish(msgJoint);
		ik_seed_state = solutions[sol_select];
}
