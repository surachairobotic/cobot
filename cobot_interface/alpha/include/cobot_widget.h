#ifndef COBOT_WIDGET_H
#define COBOT_WIDGET_H

#include <rviz/display.h>
#include <QWidget>
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/Jog.h"
#include "sensor_msgs/JointState.h"

#include "cobot_label.h"

namespace rviz
{
class DisplayContext;
}

namespace Ui {
	class CobotInterfaceAlphaUI;
}

namespace cobot_interface {
	class CobotDisplay;

	class CobotWidget : public QWidget
	{
	  friend class CobotDisplay;
			Q_OBJECT

		public:
		  CobotWidget(CobotDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = 0);
			~CobotWidget();

			void enable();
			void disable();

//		protected:
			CobotDisplay* planning_display_;
      rviz::DisplayContext* context_;
			Ui::CobotInterfaceAlphaUI *ui_;

			CobotLabel cobot_label_teach_header;

		protected:
			ros::ServiceClient srv_teach_enable;
			ros::ServiceClient srv_jog_enable;
			ros::ServiceClient srv_read_point_file;
			ros::ServiceClient srv_edit_js_file;

		private:
			std::vector<sensor_msgs::JointState> js_points;
			cobot_msgs::Jog msg_jog;

			void updatePointsTable();

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

#endif // COBOT_WIDGET_H
