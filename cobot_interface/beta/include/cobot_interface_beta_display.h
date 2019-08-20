#ifndef COBOT_INTERFACE_BETA_DISPLAY_H
#define COBOT_INTERFACE_BETA_DISPLAY_H

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#include <cobot_interface_beta_widget.h>
#include "cobot_msgs/PickPlacePoseArray.h"
#include "std_msgs/Bool.h"

#include "ui_cobot_interface_beta.h"

namespace cobot_interface
{
	class CobotInterfaceBetaDisplay : public rviz::Display
	{
		Q_OBJECT

		public:
			CobotInterfaceBetaDisplay();
			~CobotInterfaceBetaDisplay();

			void setName(const QString& name);

			static void callback_js(const sensor_msgs::JointState &js);
			static void callback_pppa(const cobot_msgs::PickPlacePoseArray &_msg);
			static void callback_update_table(const std_msgs::Bool &msg);
			sensor_msgs::JointState js;
			cobot_msgs::PickPlacePoseArray pppa;

			ros::NodeHandle nh_;

		Q_SIGNALS:
			void jsUpdate();
			void pppaUpdate();

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

			ros::AsyncSpinner spinner;

			CobotInterfaceBetaWidget *frame_;
			rviz::PanelDockWidget* frame_dock_;

		private:
			ros::Subscriber sub_js, sub_pppa, sub_update_table;

		private Q_SLOTS:
		  void cobotPanelVisibilityChange(bool enable);
	};
}
#endif // COBOT_INTERFACE_BETA_DISPLAY_H
