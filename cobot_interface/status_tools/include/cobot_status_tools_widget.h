#ifndef COBOT_STATUS_TOOLS_WIDGET_H
#define COBOT_STATUS_TOOLS_WIDGET_H

#include <rviz/display.h>
#include <QWidget>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/Jog.h"
#include "sensor_msgs/JointState.h"

namespace rviz
{
class DisplayContext;
}

namespace Ui {
	class CobotToolsUI;
}

namespace cobot_interface {
	class CobotStatusToolsDisplay;

	class CobotStatusToolsWidget : public QWidget
	{
	  friend class CobotStatusToolsDisplay;
			Q_OBJECT

		public:
		  CobotStatusToolsWidget(CobotStatusToolsDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = 0);
			~CobotStatusToolsWidget();

			void enable();
			void disable();
			geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error);

//		protected:
			CobotStatusToolsDisplay* cobot_display_;
      rviz::DisplayContext* context_;
			Ui::CobotToolsUI *ui_;

		protected:

			// ros::ServiceClient srv_teach_enable;
			// ros::ServiceClient srv_jog_enable;
			ros::ServiceClient srv_read_point_file;
			ros::ServiceClient srv_edit_js_file;
			ros::ServiceClient srv_fk;
			ros::ServiceClient srv_cobot_planning;

			//static void callback_js(const sensor_msgs::JointState &js);
			void updatePointsTable();

		private:
			std::vector<sensor_msgs::JointState> js_points;
			cobot_msgs::Jog msg_jog;
			sensor_msgs::JointState js;

//			void updateJointStateUI();

			// Jog
			ros::Publisher pub_jog;

		private Q_SLOTS:
			void addPointClicked();
			void delPointClicked();
			void readPointClicked();

			void planClicked();
			void execClicked();

			void updateJointStateUI();
	};
}

#endif // COBOT_STATUS_TOOLS_WIDGET_H
