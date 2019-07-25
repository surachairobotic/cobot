#ifndef COBOT_JOG_TEACH_WIDGET_H
#define COBOT_JOG_TEACH_WIDGET_H

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
	class CobotJogTeachUI;
}

namespace cobot_interface {
	class CobotJogTeachDisplay;

	class CobotJogTeachWidget : public QWidget
	{
	  friend class CobotJogTeachDisplay;
			Q_OBJECT

		public:
		  CobotJogTeachWidget(CobotJogTeachDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = 0);
			~CobotJogTeachWidget();

			void enable();
			void disable();
			geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error);

//		protected:
			CobotJogTeachDisplay* cobot_display_;
      rviz::DisplayContext* context_;
			Ui::CobotJogTeachUI *ui_;

		protected:

			// ros::ServiceClient srv_teach_enable;
			// ros::ServiceClient srv_jog_enable;
			ros::ServiceClient srv_read_point_file;
			ros::ServiceClient srv_edit_js_file;
			ros::ServiceClient srv_fk;
			ros::ServiceClient srv_cobot_planning;

		private:
			std::vector<sensor_msgs::JointState> js_points;
			cobot_msgs::Jog msg_jog;
			sensor_msgs::JointState js;

//			void updateJointStateUI();

			// Jog
			ros::Publisher pub_jog;

		private Q_SLOTS:

			void updateJointStateUI();
	};
}

#endif // COBOT_JOG_TEACH_WIDGET_H
