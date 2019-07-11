#ifndef COBOT_STATUS_TOOLS_DISPLAY_H
#define COBOT_STATUS_TOOLS_DISPLAY_H

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#include <cobot_status_tools_widget.h>

#include "ui_cobot_status_tools.h"

namespace cobot_interface
{
	class CobotStatusToolsDisplay : public rviz::Display
	{
		Q_OBJECT

		public:
			CobotStatusToolsDisplay();
			~CobotStatusToolsDisplay();

			void setName(const QString& name);

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

			ros::NodeHandle nh_;
			ros::AsyncSpinner spinner;

			CobotStatusToolsWidget *frame_;
      rviz::PanelDockWidget* frame_dock_;

		private:
			ros::Subscriber sub_js;
			sensor_msgs::JointState js;
			static void callback_js(const sensor_msgs::JointState &js);

		private Q_SLOTS:
		  void cobotPanelVisibilityChange(bool enable);
	};
}
#endif // COBOT_STATUS_TOOLS_DISPLAY_H
