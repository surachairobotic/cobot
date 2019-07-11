#include <cobot_widget.h>
#include "cobot_msgs/Jog.h"

#include "ui_cobot_interface_alpha.h"

using namespace cobot_interface;

void CobotWidget::jog_enableClicked() {
  ROS_INFO("jog_enableClicked");
  cobot_msgs::EnableNode srv;
  srv.request.enable = true;
  if (srv_jog_enable.call(srv))
    ROS_INFO("srv_jog_enable: %s", srv.response.error.data.c_str());
  else
    ROS_ERROR("Failed to call service srv_jog_enable");
}

void CobotWidget::upJ1Clicked()
{
	ROS_INFO("upJ1Clicked !!!");
  msg_jog.cmd = "J1";
  msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::upJ2Clicked()
{
	ROS_INFO("upJ2Clicked !!!");
  msg_jog.cmd = "J2";
  msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::upJ3Clicked()
{
	ROS_INFO("upJ3Clicked !!!");
  msg_jog.cmd = "J3";
  msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::upJ4Clicked()
{
	ROS_INFO("upJ4Clicked !!!");
  msg_jog.cmd = "J4";
  msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::upJ5Clicked()
{
	ROS_INFO("upJ5Clicked !!!");
  msg_jog.cmd = "J5";
  msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";

}
void CobotWidget::upJ6Clicked()
{
	ROS_INFO("upJ6Clicked !!!");
  msg_jog.cmd = "J6";
  msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::downJ1Clicked()
{
	ROS_INFO("downJ1Clicked !!!");
  msg_jog.cmd = "J1";
  msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::downJ2Clicked()
{
	ROS_INFO("downJ2Clicked !!!");
  msg_jog.cmd = "J2";
  msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::downJ3Clicked()
{
	ROS_INFO("downJ3Clicked !!!");
  msg_jog.cmd = "J3";
  msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::downJ4Clicked()
{
	ROS_INFO("downJ4Clicked !!!");
  msg_jog.cmd = "J4";
  msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::downJ5Clicked()
{
	ROS_INFO("downJ5Clicked !!!");
  msg_jog.cmd = "J5";
  msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotWidget::downJ6Clicked()
{
	ROS_INFO("downJ6Clicked !!!");
  msg_jog.cmd = "J6";
  msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}
