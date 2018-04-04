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

#include <rviz/display.h>
#include <QWidget>

#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>

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
//							const std::string& robot_description = "robot_description");
			//explicit MyFrame(QWidget *parent = 0);
			~MyFrame();

			void enable();
			void disable();

			std::vector<geometry_msgs::Pose> robot_pose;

//const std::vector<geometry_msgs::Pose> image
			void newLabel(const std::vector<geometry_msgs::Pose> &image) { 
				ROS_INFO("emit 1");
				Q_EMIT requestNewLabel(image); 
				ROS_INFO("emit 2");
			}

		Q_SIGNALS :    
    	void requestNewLabel(const std::vector<geometry_msgs::Pose>);

		protected:
			MyDisplay* planning_display_;
			rviz::DisplayContext* context_;
			Ui::MultiMoveDisplayUI *ui_;

			moveit::planning_interface::MoveGroupInterfacePtr move_group_;
		
		private:
			/* timer info for rate limiting */
			ros::Timer publish_timer_;
			ros::Time init_time_;
			ros::Time last_callback_time_;
			ros::Duration average_callback_duration_;
			static const ros::Duration min_delay_;
			int schedule_request_count_;
			ros::NodeHandle nh_;

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

		private Q_SLOTS:
		  // General
		  void tabChanged(int index);
		  void step_01_Clicked();
			void updateTextbox(const std::vector<geometry_msgs::Pose> image);
	};
}

#endif // MyFrame_H
