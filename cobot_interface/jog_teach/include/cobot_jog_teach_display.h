#ifndef COBOT_JOG_TEACH_DISPLAY_H
#define COBOT_JOG_TEACH_DISPLAY_H

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#include <cobot_jog_teach_widget.h>

#include "ui_cobot_jog_teach.h"

namespace cobot_interface
{
	class CobotJogTeachDisplay : public rviz::Display
	{
		Q_OBJECT

		public:
			CobotJogTeachDisplay();
			~CobotJogTeachDisplay();

			void setName(const QString& name);

			static void callback_js(const sensor_msgs::JointState &js);
			sensor_msgs::JointState js;

		Q_SIGNALS:
			void jsUpdate();

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

			CobotJogTeachWidget *frame_;
      rviz::PanelDockWidget* frame_dock_;

		private:
			ros::Subscriber sub_js;

		private Q_SLOTS:
		  void cobotPanelVisibilityChange(bool enable);
	};
}
#endif // COBOT_JOG_TEACH_DISPLAY_H
