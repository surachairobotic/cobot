#ifndef COBOT_DISPLAY_H
#define COBOT_DISPLAY_H

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#include <cobot_widget.h>

#include "ui_cobot_interface_alpha.h"

namespace cobot_interface
{
	class CobotDisplay : public rviz::Display
	{
		Q_OBJECT

		public:
			CobotDisplay();
			~CobotDisplay();

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

			CobotWidget *frame_;
      rviz::PanelDockWidget* frame_dock_;

		private Q_SLOTS:
		  void cobotPanelVisibilityChange(bool enable);
	};
}
#endif
