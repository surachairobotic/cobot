#ifndef MYDISPLAY_H
#define MYDISPLAY_H

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>

#include <MyFrame.h>

#include "interactive_robot.h"
#include "pose_string.h"
//#include "imarker.h"

namespace my_plugin
{
	class MyDisplay : public rviz::Display
	{
		Q_OBJECT

		public:
			MyDisplay();
			~MyDisplay();

		  void setName(const QString& name);

//			static std::vector<geometry_msgs::Pose> robot_pose;
			InteractiveRobot* irobot_start_state;

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

			std::string double2string(double x);

		private:
			MyFrame *frame_;
			rviz::PanelDockWidget* frame_dock_;
			QString debug_message;

			ros::NodeHandle nh_;
			ros::Subscriber robot_start_;
			ros::Subscriber robot_goal_;

//			InteractiveRobot irobot_goal_state;
			static const std::string& FRAME_ID;
  		static const std::string PLANNING_GROUP;

//			robot_state::RobotStatePtr robot_state_;
//		  const robot_model::JointModelGroup* group_;
//			std::string end_link;
//		  Eigen::Affine3d desired_group_end_link_pose_;

			ros::NodeHandle node_handle;
			ros::AsyncSpinner spinner;

			static void callbackRobotStartState(InteractiveRobot &robot);
//			static void callbackRobotGoalState(InteractiveRobot &robot);

		private Q_SLOTS:
		  void motionPanelVisibilityChange(bool enable);
	};
}
#endif
