#ifndef MyFrame_H
#define MyFrame_H

//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include "sensor_msgs/JointState.h"

#include <rviz/display.h>
#include <QWidget>
#include <QLineEdit>

#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>

#include "cobot_kinematic.h"

namespace rviz
{
class DisplayContext;
}

namespace Ui {
	class MultiMoveDisplayUI;
}

namespace my_plugin {
	class MyDisplay;

	class MyFrame : public QWidget
	{
	  friend class MyDisplay;
			Q_OBJECT

		public:
		  MyFrame(MyDisplay* pdisplay, 
							rviz::DisplayContext* context, 
							QWidget* parent = 0);
			~MyFrame();

			void enable();
			void disable();

			void newLabel(const std::vector<geometry_msgs::Pose> &image) { 
				ROS_INFO("emit 1");
				Q_EMIT requestNewLabel(image); 
				ROS_INFO("emit 2");
			}

			CobotKinematic coco;

			bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const geometry_msgs::Pose& pose, 
												 unsigned int attempts = 0,
                			   double timeout = 0.0, 
												 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 1
/*		  bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const geometry_msgs::Pose& pose, 
												 const std::string& tip,
                 				 unsigned int attempts = 0, 
												 double timeout = 0.0,
                 				 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 2
		  bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const Eigen::Affine3d& pose, 
												 unsigned int attempts = 0,
                 				 double timeout = 0.0, 
				                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 3
		  bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const Eigen::Affine3d& pose, 
												 const std::string& tip,
                 				 unsigned int attempts = 0, 
												 double timeout = 0.0,
			                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 4
  		bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const Eigen::Affine3d& pose, 
												 const std::string& tip,
                 				 const std::vector<double>& consistency_limits, 
												 unsigned int attempts = 0, 
												 double timeout = 0.0,
			                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 5
		  bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const EigenSTL::vector_Affine3d& poses,
                 				 const std::vector<std::string>& tips, 
												 unsigned int attempts = 0, 
												 double timeout = 0.0,
				                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 6
		  bool getSolutionIK(const robot_state::JointModelGroup* group, 
												 const EigenSTL::vector_Affine3d& poses,
                 				 const std::vector<std::string>& tips, 
												 const std::vector<std::vector<double> >& consistency_limits,
                 				 unsigned int attempts = 0, 
												 double timeout = 0.0,
			                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()); // getSolutionIK 7
*/

			void best_solution(std::vector<double>& ik_seed_state, std::vector<std::vector<double>>& solutions);
			void pub_jog_robot_state(geometry_msgs::Pose pose);
			void pubCurrentRobotState(std::vector<double>& vPosition);
			void pubUiSeed(const std::vector<double>& vPosition);

			const std::string PLANNING_GROUP_;
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//			const robot_state::JointModelGroup *joint_model_group;
//			kinematics::KinematicsBaseConstPtr solver;
			std::vector<std::vector<double>> solutions;
			std::vector<double> solution;
			int iSol = 0;
//			std::string end_link;
			void inPose(geometry_msgs::Pose pose);
			geometry_msgs::Pose getEEFpose();
//			kinematic = joint_model_group->getSolverInstance();
			std::vector<geometry_msgs::Pose> robot_start_pose;
			geometry_msgs::Pose robot_goal_pose;
			robot_state::RobotStatePtr start_state;
			robot_state::RobotStatePtr goal_state;

			moveit_msgs::RobotTrajectory trajectory_1;
			moveit_msgs::RobotTrajectory trajectory_2;
			moveit_msgs::DisplayTrajectory dis_traj_1;
			moveit_msgs::DisplayTrajectory dis_traj_2;
			moveit_msgs::DisplayTrajectory dis_traj_3;

			ros::Publisher pub_plan_1;
			ros::Publisher pub_plan_2;
			ros::Publisher pub_plan_3;

		Q_SIGNALS :    
    	void requestNewLabel(const std::vector<geometry_msgs::Pose>);

		protected:
			MyDisplay* planning_display_;
			rviz::DisplayContext* context_;
			Ui::MultiMoveDisplayUI *ui_;
		
			moveit::planning_interface::MoveGroupInterfacePtr move_group_;
		  moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_1;
		  moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_2;

		Q_SIGNALS:
			void planningFinished();

		private:
			// Planning tab
			void computePlanButtonClicked();
			void configureForPlanning();
		  void computeExecuteButtonClicked();
		  void computeStopButtonClicked();

			/* timer info for rate limiting */
			ros::Timer publish_timer_;
			ros::Time init_time_;
			ros::Time last_callback_time_;
			ros::Duration average_callback_duration_;
			static const ros::Duration min_delay_;
			int schedule_request_count_;

			ros::NodeHandle nh_;
		  ros::Publisher joint_state_pub;
		  ros::Publisher joint_debug_publisher_;
			//sensor_msgs::JointState msgJoint;

			std::vector<QLineEdit*> ui_textbox;

			/* Indicate that the world or the robot has changed and
			 * the new state needs to be updated and published to rviz */
			void scheduleUpdate();

			/* set the callback timer to fire if needed.
			 * Return true if callback should happen immediately. */
			bool setCallbackTimer(bool new_update_request);

		  /* update the world and robot state and publish to rviz */
		  void updateCallback(const ros::TimerEvent& e);

		  /* functions to calculate new state and publish to rviz */
		  void updateAll();

			void configureWorkspace();
			void changePlanningGroupHelper();

		private Q_SLOTS:
		  // General
		  void tabChanged(int index);
		  void step_01_Clicked();

			// Joint
			void upJ1Clicked();
			void upJ2Clicked();
			void upJ3Clicked();
			void upJ4Clicked();
			void upJ5Clicked();
			void upJ6Clicked();
			void downJ1Clicked();
			void downJ2Clicked();
			void downJ3Clicked();
			void downJ4Clicked();
			void downJ5Clicked();
			void downJ6Clicked();
			// EEF
			void downXClicked();
			void downYClicked();
			void downZClicked();
			void downRXClicked();
			void downRYClicked();
			void downRZClicked();
			void upXClicked();
			void upYClicked();
			void upZClicked();
			void upRXClicked();
			void upRYClicked();
			void upRZClicked();

			void pushButtonClicked();
			void pushButton_2Clicked();
			void btn_inputClicked();

			void updateTextbox(const std::vector<geometry_msgs::Pose> image);

			// Move tab
			void planButtonClicked();
		  void executeButtonClicked();
			void onFinishedExecution(bool success);
		  void stopButtonClicked();
	};
}

#endif // MyFrame_H
