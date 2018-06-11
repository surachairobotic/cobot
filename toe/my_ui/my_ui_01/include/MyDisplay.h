#ifndef MYDISPLAY_H
#define MYDISPLAY_H

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit/background_processing/background_processing.h>


#include <MyFrame.h>

#include "interactive_robot.h"
#include "pose_string.h"

namespace my_plugin
{
	class MyDisplay : public rviz::Display
	{
		Q_OBJECT

		public:
			MyDisplay();
			~MyDisplay();

			/* robot info */
			const std::string robot_description = "robot_description";
			robot_model_loader::RobotModelLoader rm_loader_;
		  robot_model::RobotModelPtr robot_model_;

			/* info about joint group we are manipulating */
			const std::string planning_group = "arm";
			const robot_model::JointModelGroup* group_;
			std::string end_link;
			Eigen::Affine3d desired_group_end_link_pose_;
			kinematics::KinematicsBaseConstPtr solver;

			/** Queue this function call for execution within the background thread
				  All jobs are queued and processed in order by a single background thread. */
			void addBackgroundJob(const boost::function<void()>& job, const std::string& name);

			/** Directly spawn a (detached) background thread for execution of this function call
				  Should be used, when order of processing is not relevant / job can run in parallel.
				  Must be used, when job will be blocking. Using addBackgroundJob() in this case will block other queued jobs as
				 well */
			void spawnBackgroundJob(const boost::function<void()>& job);

			/// queue the execution of this function for the next time the main update() loop gets called
			void addMainLoopJob(const boost::function<void()>& job);

			void waitForAllMainLoopJobs();

			/// remove all queued jobs
			void clearJobs();

			/** exception thrown when a problem occurs */
			class RobotLoadException : std::exception
			{
			};

		  void setName(const QString& name);
			void updateUIJointsPosition();

			robot_state::RobotStatePtr robotCurrentState;
			sensor_msgs::JointState jointCurrentState;
			std::vector<std::string> sJointsName;

			InteractiveRobot* irobot_start_state;
			InteractiveRobot* irobot_goal_state;

		protected:
			virtual void load(const rviz::Config& config);
			virtual void save(rviz::Config config) const;

			virtual void update(float wall_dt, float ros_dt);
			virtual void reset();

			// overrides from Display
			virtual void onInitialize();
			virtual void onEnable();
			virtual void onDisable();
			virtual void fixedFrameChanged();

		  moveit::tools::BackgroundProcessing background_process_;
		  std::deque<boost::function<void()> > main_loop_jobs_;
			boost::mutex main_loop_jobs_lock_;
			boost::condition_variable main_loop_jobs_empty_condition_;

			MyFrame *frame_;
			rviz::PanelDockWidget* frame_dock_;
			QString debug_message;

			ros::NodeHandle nh_;
		  ros::Publisher robot_state_publisher_;
			ros::Subscriber joint_state_subscriber_;
//			static bool in_callback;

			static const std::string& FRAME_ID;
  		static const std::string PLANNING_GROUP;

			ros::NodeHandle node_handle;
			ros::AsyncSpinner spinner;

			void updateUIJointsLabel();

			static void callbackRobotStartState(InteractiveRobot &robot);
			static void callbackRobotGoalState(InteractiveRobot &robot);
			static void callbackJointState(const sensor_msgs::JointState& msgJoint);

			void updateCurrentRobotState();

		private Q_SLOTS:
		  void motionPanelVisibilityChange(bool enable);

/*
			void btn_x_inc_Clicked();
			void btn_x_dec_Clicked();
			void btn_y_inc_Clicked();
			void btn_y_dec_Clicked();
			void btn_z_inc_Clicked();
			void btn_z_dec_Clicked();
			void btn_rx_inc_Clicked();
			void btn_rx_dec_Clicked();
			void btn_ry_inc_Clicked();
			void btn_ry_dec_Clicked();
			void btn_rz_inc_Clicked();
			void btn_rz_dec_Clicked();
*/
	};
}
#endif
