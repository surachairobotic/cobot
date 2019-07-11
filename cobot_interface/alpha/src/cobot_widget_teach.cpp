#include <cobot_widget.h>
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "cobot_msgs/ReadJointStateFile.h"

#include "ui_cobot_interface_alpha.h"

using namespace cobot_interface;

void CobotWidget::enableNodeClicked() {
  ROS_INFO("enableNodeClicked");
  cobot_msgs::EnableNode srv;
  srv.request.enable = true;
  if (srv_teach_enable.call(srv))
    ROS_INFO("srv_teach_enable: %s", srv.response.error.data.c_str());
  else
    ROS_ERROR("Failed to call service srv_teach_enable");
}
void CobotWidget::disableNodeClicked() {
  ROS_INFO("disableNodeClicked");
  cobot_msgs::EnableNode srv;
  srv.request.enable = false;
  if (srv_teach_enable.call(srv))
    ROS_INFO("srv_teach_enable: %s", srv.response.error.data.c_str());
  else
    ROS_ERROR("Failed to call service srv_teach_enable");
}
void CobotWidget::pointsNodeClicked() {
  ROS_INFO("pointsNodeClicked");
	cobot_msgs::ReadJointStateFile req_read_point_file;
	if (srv_read_point_file.call(req_read_point_file))
		ROS_INFO("srv_read_point_file.call is true");
	else
		ROS_INFO("srv_read_point_file.call is false");
	ROS_INFO("req_read_point_file.response.error.data : %s", req_read_point_file.response.error.data.c_str());
	if (req_read_point_file.response.error.data == "OK") {
		js_points = req_read_point_file.response.js;
		updatePointsTable();
	}
}

void CobotWidget::addPointClicked() {
	ROS_INFO("CobotWidget::addPointClicked()");
	ROS_INFO("current row : %d", ui_->list_teach_points->currentRow());
	cobot_msgs::EditJointStateFile req_edit_js_file;
	req_edit_js_file.request.index = ui_->list_teach_points->currentRow();
	req_edit_js_file.request.operation = 1;
	if (srv_edit_js_file.call(req_edit_js_file))
		ROS_INFO("srv_edit_js_file.call is true");
	else
		ROS_INFO("srv_edit_js_file.call is false");
	ROS_INFO("req_edit_js_file.response.error.data : %s", req_edit_js_file.response.error.data.c_str());
	if (req_edit_js_file.response.error.data == "OK") ;
	updatePointsTable();
}
void CobotWidget::delPointClicked() {
	ROS_INFO("CobotWidget::delPointClicked()");
	ROS_INFO("current row : %d", ui_->list_teach_points->currentRow());
	cobot_msgs::EditJointStateFile req_edit_js_file;
	req_edit_js_file.request.index = ui_->list_teach_points->currentRow();
	req_edit_js_file.request.operation = -1;
	if (srv_edit_js_file.call(req_edit_js_file))
		ROS_INFO("srv_edit_js_file.call is true");
	else
		ROS_INFO("srv_edit_js_file.call is false");
	ROS_INFO("req_edit_js_file.response.error.data : %s", req_edit_js_file.response.error.data.c_str());
	if (req_edit_js_file.response.error.data == "OK") ;
	updatePointsTable();
}

void CobotWidget::updatePointsTable() {
	ROS_INFO("CobotWidget::updatePointsTable()");
	ui_->list_teach_points->clear();
  for(int i=0; i<js_points.size(); i++) {
		ui_->list_teach_points->addItem(QString::number(i));
	}
}
