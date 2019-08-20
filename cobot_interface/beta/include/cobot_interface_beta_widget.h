#ifndef COBOT_INTERFACE_BETA_WIDGET_H
#define COBOT_INTERFACE_BETA_WIDGET_H

#include <rviz/display.h>
#include <QWidget>
#include <QPushButton>
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
	class CobotInterfaceBetaUI;
}

namespace cobot_interface {
	class CobotInterfaceBetaDisplay;

	class CobotInterfaceBetaWidget : public QWidget
	{
	  friend class CobotInterfaceBetaDisplay;
			Q_OBJECT

		public:
		  CobotInterfaceBetaWidget(CobotInterfaceBetaDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = 0);
			~CobotInterfaceBetaWidget();

			void enable();
			void disable();
			geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error);
			bool jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, geometry_msgs::Pose &ps);
			void updatePointsTable();

//		protected:
			CobotInterfaceBetaDisplay* cobot_display_;
      rviz::DisplayContext* context_;
			Ui::CobotInterfaceBetaUI *ui_;

		protected:
			ros::ServiceClient srv_teach_enable;
			ros::ServiceClient srv_jog_enable;
			ros::ServiceClient srv_read_point_file;
			ros::ServiceClient srv_edit_js_file;
			ros::ServiceClient srv_fk;
			ros::ServiceClient srv_cobot_planning;

		private:
			std::vector<sensor_msgs::JointState> js_points;
			cobot_msgs::Jog msg_jog;
			sensor_msgs::JointState js;
			bool teach_status, jog_status, pick_status;
//			void updateJointStateUI();

			// Jog
			ros::Publisher pub_jog, pub_pick_en, pub_pick_cmd, pub_stop;
			void changeColor(QPushButton* button, int color);

		private Q_SLOTS:
			void updateJointStateUI();
			void updatePPPAUI();

			void enableTeachClicked();

			void enableJogClicked();
			void jogHandle(const std::string &j, int mode);

			void editPointClicked(bool mode);

			void planClicked();
			void execClicked();
			void stopClicked();

			void pick_en_Clicked();
			void pickHandle(const std::string &msg);
	};
}

#endif // COBOT_INTERFACE_BETA_WIDGET_H
