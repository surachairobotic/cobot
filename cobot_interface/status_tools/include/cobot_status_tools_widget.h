#ifndef COBOT_STATUS_TOOLS_WIDGET_H
#define COBOT_STATUS_TOOLS_WIDGET_H

#include <rviz/display.h>
#include <QWidget>
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

//		protected:
			CobotStatusToolsDisplay* planning_display_;
      rviz::DisplayContext* context_;
			Ui::CobotToolsUI *ui_;

		protected:
			
			ros::ServiceClient srv_teach_enable;
			ros::ServiceClient srv_jog_enable;
			ros::ServiceClient srv_read_point_file;
			ros::ServiceClient srv_edit_js_file;

			//static void callback_js(const sensor_msgs::JointState &js);

		private:
			std::vector<sensor_msgs::JointState> js_points;
			cobot_msgs::Jog msg_jog;
			sensor_msgs::JointState js;

			void updatePointsTable();
			void updateJointStateUI();

			// Jog
			ros::Publisher pub_jog;

		private Q_SLOTS:

			// teach
			void enableNodeClicked();
			void disableNodeClicked();
			void pointsNodeClicked();
			void addPointClicked();
			void delPointClicked();

			// Jog
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
			void jog_enableClicked();
	};
}

#endif // COBOT_STATUS_TOOLS_WIDGET_H
