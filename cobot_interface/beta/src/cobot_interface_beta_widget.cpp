#include <cobot_interface_beta_widget.h>
#include <cobot_interface_beta_display.h>

#include <QResizeEvent>
#include <QSize>
#include <QRect>
#include <QPushButton>
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/GetPositionFK.h"
#include "cobot_planner/CobotPlanning.h"
#include "tf/tf.h"
#include <actionlib/client/simple_action_client.h>
#include "cobot_msgs/ExecuteAction.h"
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/ReadJointStateFile.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "std_msgs/Bool.h"

#include "ui_cobot_interface_beta.h"

using namespace cobot_interface;

CobotInterfaceBetaWidget::CobotInterfaceBetaWidget( CobotInterfaceBetaDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent)
	: cobot_display_(pdisplay)
  , context_(context)
  , QWidget(parent)
  , ui_(new Ui::CobotInterfaceBetaUI)
{
  ui_->setupUi(this);

	ui_->lineEdit_res->setText(QString::number(0.025, 'f', 2));
	ui_->lineEdit_velo->setText(QString::number(0.1, 'f', 2));
	teach_status = jog_status = pick_status = false;
	changeColor(ui_->btn_teach, -1);
	changeColor(ui_->btn_jog, -1);

	connect(cobot_display_, SIGNAL(jsUpdate()), this, SLOT(updateJointStateUI()));
	connect(cobot_display_, SIGNAL(pppaUpdate()), this, SLOT(updatePPPAUI()));
	connect(ui_->btn_teach, SIGNAL(clicked()), this, SLOT(enableTeachClicked()));

	// jog_mode
	connect(ui_->btn_jog, SIGNAL(clicked()), this, SLOT(enableJogClicked()));
	connect(ui_->btn_home, &QPushButton::clicked, this, [this]{ jogHandle("HOME", 0); });
	connect(ui_->btn_j1_p, &QPushButton::clicked, this, [this]{ jogHandle("J1", 1); });
	connect(ui_->btn_j2_p, &QPushButton::clicked, this, [this]{ jogHandle("J2", 1); });
	connect(ui_->btn_j3_p, &QPushButton::clicked, this, [this]{ jogHandle("J3", 1); });
	connect(ui_->btn_j4_p, &QPushButton::clicked, this, [this]{ jogHandle("J4", 1); });
	connect(ui_->btn_j5_p, &QPushButton::clicked, this, [this]{ jogHandle("J5", 1); });
	connect(ui_->btn_j6_p, &QPushButton::clicked, this, [this]{ jogHandle("J6", 1); });
	connect(ui_->btn_j1_m, &QPushButton::clicked, this, [this]{ jogHandle("J1", -1); });
	connect(ui_->btn_j2_m, &QPushButton::clicked, this, [this]{ jogHandle("J2", -1); });
	connect(ui_->btn_j3_m, &QPushButton::clicked, this, [this]{ jogHandle("J3", -1); });
	connect(ui_->btn_j4_m, &QPushButton::clicked, this, [this]{ jogHandle("J4", -1); });
	connect(ui_->btn_j5_m, &QPushButton::clicked, this, [this]{ jogHandle("J5", -1); });
	connect(ui_->btn_j6_m, &QPushButton::clicked, this, [this]{ jogHandle("J6", -1); });

	connect(ui_->btn_x_p, &QPushButton::clicked, this, [this]{ jogHandle("CX", 1); });
	connect(ui_->btn_y_p, &QPushButton::clicked, this, [this]{ jogHandle("CY", 1); });
	connect(ui_->btn_z_p, &QPushButton::clicked, this, [this]{ jogHandle("CZ", 1); });
	connect(ui_->btn_rx_p, &QPushButton::clicked, this, [this]{ jogHandle("CR", 1); });
	connect(ui_->btn_ry_p, &QPushButton::clicked, this, [this]{ jogHandle("CP", 1); });
	connect(ui_->btn_rz_p, &QPushButton::clicked, this, [this]{ jogHandle("CY", 1); });
	connect(ui_->btn_x_m, &QPushButton::clicked, this, [this]{ jogHandle("CX", -1); });
	connect(ui_->btn_y_m, &QPushButton::clicked, this, [this]{ jogHandle("CY", -1); });
	connect(ui_->btn_z_m, &QPushButton::clicked, this, [this]{ jogHandle("CZ", -1); });
	connect(ui_->btn_rx_m, &QPushButton::clicked, this, [this]{ jogHandle("CR", -1); });
	connect(ui_->btn_ry_m, &QPushButton::clicked, this, [this]{ jogHandle("CP", -1); });
	connect(ui_->btn_rz_m, &QPushButton::clicked, this, [this]{ jogHandle("CW", -1); });

	connect(ui_->btn_add, &QPushButton::clicked, this, [this]{ editPointClicked(true); });
	connect(ui_->btn_del, &QPushButton::clicked, this, [this]{ editPointClicked(false); });

	connect(ui_->btn_plan, SIGNAL(clicked()), this, SLOT(planClicked()));
	connect(ui_->btn_exec, SIGNAL(clicked()), this, SLOT(execClicked()));

	connect(ui_->btn_pick, SIGNAL(clicked()), this, SLOT(pick_en_Clicked()));
	connect(ui_->btn_m1, &QPushButton::clicked, this, [this]{ pickHandle("M1"); });
	connect(ui_->btn_m2, &QPushButton::clicked, this, [this]{ pickHandle("M2"); });
}

CobotInterfaceBetaWidget::~CobotInterfaceBetaWidget() {
	ROS_INFO("CobotInterfaceBetaWidget::~CobotInterfaceBetaWidget()");
  delete ui_;
}

void CobotInterfaceBetaWidget::enable() {
  // activate the frame
  parentWidget()->show();
  show();
}

void CobotInterfaceBetaWidget::disable() {
  parentWidget()->hide();
  hide();
}

/*
void CobotInterfaceBetaWidget::resizeEvent(QResizeEvent *event) {
	QSize size = event->size();
	QRect rect(0, 0, size.width(), size.height());
	ui_->gridLayout->setGeometry(rect);
	QRect tmp = ui_->gridLayout->geometry();
//	ROS_INFO("CobotInterfaceBetaWidget::resizeEvent : [%d, %d][%d, %d, %d, %d]", size.width(), size.height(), tmp.x(), tmp.y(), tmp.width(), tmp.height());
}
*/

void CobotInterfaceBetaWidget::updateJointStateUI() {
//	ROS_INFO("CobotInterfaceBetaWidget::updateJointStateUI");
	ui_->lineEdit_j1->setText(QString::number(cobot_display_->js.position[0], 'f', 4));
	ui_->lineEdit_j2->setText(QString::number(cobot_display_->js.position[1], 'f', 4));
	ui_->lineEdit_j3->setText(QString::number(cobot_display_->js.position[2], 'f', 4));
	ui_->lineEdit_j4->setText(QString::number(cobot_display_->js.position[3], 'f', 4));
	ui_->lineEdit_j5->setText(QString::number(cobot_display_->js.position[4], 'f', 4));
	ui_->lineEdit_j6->setText(QString::number(cobot_display_->js.position[5], 'f', 4));

	std::vector<double> rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::string error_code = "";
	geometry_msgs::Pose cartesian = jsCartesian(cobot_display_->js, rpy, error_code);
	if(error_code == "OK") {
		ui_->lineEdit_x->setText(QString::number(cartesian.position.x, 'f', 4));
		ui_->lineEdit_y->setText(QString::number(cartesian.position.y, 'f', 4));
		ui_->lineEdit_z->setText(QString::number(cartesian.position.z, 'f', 4));
		ui_->lineEdit_rx->setText(QString::number(rpy[3], 'f', 4));
		ui_->lineEdit_ry->setText(QString::number(rpy[4], 'f', 4));
		ui_->lineEdit_rz->setText(QString::number(rpy[5], 'f', 4));
	}
}

void CobotInterfaceBetaWidget::updatePPPAUI() {
	std::string str_pick = "";
	std::string str_place = "";
	for(int i=0; i<3; i++) {
		for(int j=0; j<cobot_display_->pppa.data.size(); j++) {
			if(cobot_display_->pppa.data[j].b_pick == i)
				str_pick += cobot_display_->pppa.data[j].label + ", ";
			if(cobot_display_->pppa.data[j].b_place == i)
				str_place += cobot_display_->pppa.data[j].label + ", ";
		}
	}
	ui_->label_cam_a->setText(QString::fromStdString(str_pick));
	ui_->label_cam_b->setText(QString::fromStdString(str_place));
}

geometry_msgs::Pose CobotInterfaceBetaWidget::jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error) {
	moveit_msgs::GetPositionFK msg;
	msg.request.header.stamp = ros::Time::now();
	msg.request.fk_link_names = {"tool0"};
	msg.request.robot_state.joint_state = _js;
	error = "OK";
	srv_fk = cobot_display_->nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
	if(srv_fk.call(msg))
	{
		_pose[0] = msg.response.pose_stamped[0].pose.position.x;
		_pose[1] = msg.response.pose_stamped[0].pose.position.y;
		_pose[2] = msg.response.pose_stamped[0].pose.position.z;
		tf::Quaternion q_ori;
		tf::quaternionMsgToTF(msg.response.pose_stamped[0].pose.orientation , q_ori);
		tf::Matrix3x3 m(q_ori);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		_pose[3] = roll;
		_pose[4] = pitch;
		_pose[5] = yaw;
	}
	else {
		ROS_ERROR("Failed to call service compute_fk");
		error = "Failed to call service compute_fk";
	}
	return msg.response.pose_stamped[0].pose;
}

void CobotInterfaceBetaWidget::enableTeachClicked() {
  cobot_msgs::EnableNode srv;
  srv.request.enable = !teach_status;
	srv_teach_enable = cobot_display_->nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_teach/enable");
  if (srv_teach_enable.call(srv)) {
		ROS_INFO("srv_teach_enable: %s", srv.response.error.data.c_str());
		teach_status = !teach_status;
	}
  else
    ROS_ERROR("Failed to call service srv_teach_enable");
	changeColor(ui_->btn_teach, teach_status ? 1 : -1);
}

void CobotInterfaceBetaWidget::changeColor(QPushButton* button, int color) {
	QPalette pal = button->palette();
	if(color == -1)
		pal.setColor(QPalette::Button, QColor(Qt::red));
	else if(color == 1)
		pal.setColor(QPalette::Button, QColor(Qt::green));
	button->setAutoFillBackground(true);
	button->setPalette(pal);
	button->update();
}

void CobotInterfaceBetaWidget::enableJogClicked() {
  ROS_INFO("enableJogClicked");
	cobot_msgs::EnableNode srv;
  srv.request.enable = !jog_status;
	srv_jog_enable = cobot_display_->nh_.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_jog/enable");
  if (srv_jog_enable.call(srv)) {
		ROS_INFO("srv_jog_enable: %s", srv.response.error.data.c_str());
		jog_status = !jog_status;
	}
  else
    ROS_ERROR("Failed to call service srv_jog_enable");
	changeColor(ui_->btn_jog, jog_status ? 1 : -1);
}

void CobotInterfaceBetaWidget::jogHandle(const std::string &j, int mode) {
	msg_jog.cmd = j;
	if(mode == 1)
		msg_jog.resolution = ui_->lineEdit_res->text().toDouble();
	else if(mode == -1)
		msg_jog.resolution = -ui_->lineEdit_res->text().toDouble();
	msg_jog.velocity = ui_->lineEdit_velo->text().toDouble();
  pub_jog.publish(msg_jog);
  msg_jog.cmd = "";
}

void CobotInterfaceBetaWidget::editPointClicked(bool mode) {
	ROS_INFO("CobotInterfaceBetaWidget::addPointClicked()");
	ROS_INFO("current row : %d", ui_->listWidget_points->currentRow());
	cobot_msgs::EditJointStateFile req_edit_js_file;
	req_edit_js_file.request.index = ui_->listWidget_points->currentRow();
	if(mode)
		req_edit_js_file.request.operation = 1;
	else
		req_edit_js_file.request.operation = -1;

	srv_edit_js_file = cobot_display_->nh_.serviceClient<cobot_msgs::EditJointStateFile>("/cobot/cobot_core/edit_js_file");
	if (srv_edit_js_file.call(req_edit_js_file))
		ROS_INFO("srv_edit_js_file.call is true");
	else
		ROS_INFO("srv_edit_js_file.call is false");
	ROS_INFO("req_edit_js_file.response.error.data : %s", req_edit_js_file.response.error.data.c_str());
	if (req_edit_js_file.response.error.data == "OK")
		updatePointsTable();
}

void CobotInterfaceBetaWidget::updatePointsTable() {
	ROS_INFO("CobotInterfaceBetaWidget::updatePointsTable()");

	cobot_msgs::ReadJointStateFile req_read_point_file;
	srv_read_point_file = cobot_display_->nh_.serviceClient<cobot_msgs::ReadJointStateFile>("/cobot/cobot_core/read_js_file");
	if (srv_read_point_file.call(req_read_point_file))
		ROS_INFO("srv_read_point_file.call is true");
	else {
		ROS_INFO("srv_read_point_file.call is false");
		return;
	}
	ROS_INFO("req_read_point_file.response.error.data : %s", req_read_point_file.response.error.data.c_str());
	if (req_read_point_file.response.error.data == "OK") {
		js_points = req_read_point_file.response.js;
//		updatePointsTable();
		ui_->listWidget_points->clear();
	  for(int i=0; i<js_points.size(); i++) {

			std::vector<double> rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			std::string error_code = "";
			geometry_msgs::Pose cartesian = jsCartesian(js_points[i], rpy, error_code);
			if(error_code == "OK") {
				QString str;
				str.sprintf("P%d| X:%.4lf, Y:%.4lf, Z:%.4lf", i, cartesian.position.x, cartesian.position.y, cartesian.position.z);
				ui_->listWidget_points->addItem(str);
			}
		}
	}
}

void CobotInterfaceBetaWidget::planClicked() {
	ROS_INFO("CobotInterfaceBetaWidget::planClicked");
	cobot_planner::CobotPlanning req_plan;

	for(int i=0; i<js_points.size(); i++) {
		std::vector<double> rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		std::string error_code = "";
		geometry_msgs::Pose cartesian = jsCartesian(js_points[i], rpy, error_code);
		if(error_code == "OK")
			req_plan.request.pose_array.push_back(cartesian);
		else
			ROS_INFO("cartesian pose is false");
	}

	req_plan.request.joint_names = {"J1", "J2", "J3", "J4", "J5", "J6"};
	req_plan.request.type = "line";
	req_plan.request.max_velocity = 0.1;
	req_plan.request.max_acceleration = 0.1;
	req_plan.request.step_time = 0.01;

	srv_cobot_planning = cobot_display_->nh_.serviceClient<cobot_planner::CobotPlanning>("/cobot/planning");
	if (srv_cobot_planning.call(req_plan))
		ROS_INFO("srv_cobot_planning.call is true");
	else
		ROS_INFO("srv_cobot_planning.call is false");

	ROS_INFO("req_plan.response.error_code : %d", req_plan.response.error_code);
	if (req_plan.response.error_code == 0) {
		;
	}
}
void CobotInterfaceBetaWidget::execClicked() {
	ROS_INFO("CobotInterfaceBetaWidget::execClicked");
	actionlib::SimpleActionClient<cobot_msgs::ExecuteAction> ac("cobot_execute", true);
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  cobot_msgs::ExecuteGoal goal;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
	ROS_INFO("execClicked is done.");
}

void CobotInterfaceBetaWidget::pick_en_Clicked() {
	ROS_INFO("CobotInterfaceBetaWidget::pick_en_Clicked");
	std_msgs::Bool msg;
	pick_status = !pick_status;
	msg.data = pick_status;
	pub_pick_en.publish(msg);
}

void CobotInterfaceBetaWidget::pickHandle(const std::string &msg) {
	ROS_INFO("CobotInterfaceBetaWidget::pickHandle");
	std_msgs::String cmd;
	cmd.data = msg;
	pub_pick_cmd.publish(cmd);
}
