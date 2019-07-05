#ifndef COBOT_WIDGET_H
#define COBOT_WIDGET_H

#include <rviz/display.h>
#include <QWidget>
#include "cobot_msgs/EnableNode.h"
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
			ros::ServiceClient srv_read_point_file;
			ros::ServiceClient srv_edit_js_file;

		private:
			std::vector<sensor_msgs::JointState> js_points;

			void updatePointsTable();

		private Q_SLOTS:
			void enableNodeClicked();
			void disableNodeClicked();
			void pointsNodeClicked();
			void addPointClicked();
			void delPointClicked();
	};
}

#endif // COBOT_WIDGET_H
