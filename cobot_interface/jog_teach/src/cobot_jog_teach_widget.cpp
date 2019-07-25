#include <cobot_jog_teach_widget.h>
#include <cobot_jog_teach_display.h>

#include <QResizeEvent>
#include <QSize>
#include <QRect>
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/GetPositionFK.h"
#include "cobot_planner/CobotPlanning.h"
#include "tf/tf.h"
#include <actionlib/client/simple_action_client.h>
#include "cobot_msgs/ExecuteAction.h"


#include "ui_cobot_jog_teach.h"

using namespace cobot_interface;

CobotJogTeachWidget::CobotJogTeachWidget( CobotJogTeachDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent)
	: cobot_display_(pdisplay)
  , context_(context)
  , QWidget(parent)
  , ui_(new Ui::CobotJogTeachUI)
//  , cobot_label_teach_header(ui_->label_teach_header)
{
  ui_->setupUi(this);

/*
	connect(cobot_display_, SIGNAL(jsUpdate()), this, SLOT(updateJointStateUI()));

	connect(ui_->btn_add, SIGNAL(clicked()), this, SLOT(addPointClicked()));
	connect(ui_->btn_del, SIGNAL(clicked()), this, SLOT(delPointClicked()));
	connect(ui_->btn_plan, SIGNAL(clicked()), this, SLOT(planClicked()));
	connect(ui_->btn_exec, SIGNAL(clicked()), this, SLOT(execClicked()));
*/
//  srv_teach_enable = n.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_teach");
//  CobotLabel cobot_label_teach_header;
//  connect(cobot_label_teach_header, SIGNAL(clicked()), this, SLOT(enableNodeClicked()));

/*
	// teach_mode
  connect(ui_->btn_teach_enable, SIGNAL(clicked()), this, SLOT(enableNodeClicked()));
	connect(ui_->btn_teach_disable, SIGNAL(clicked()), this, SLOT(disableNodeClicked()));
	connect(ui_->btn_teach_add, SIGNAL(clicked()), this, SLOT(addPointClicked()));
	connect(ui_->btn_teach_del, SIGNAL(clicked()), this, SLOT(delPointClicked()));
	connect(ui_->btn_teach_points, SIGNAL(clicked()), this, SLOT(pointsNodeClicked()));

	// jog_mode
	connect(ui_->btnUpJ1, SIGNAL(clicked()), this, SLOT(upJ1Clicked()));
	connect(ui_->btnUpJ2, SIGNAL(clicked()), this, SLOT(upJ2Clicked()));
	connect(ui_->btnUpJ3, SIGNAL(clicked()), this, SLOT(upJ3Clicked()));
	connect(ui_->btnUpJ4, SIGNAL(clicked()), this, SLOT(upJ4Clicked()));
	connect(ui_->btnUpJ5, SIGNAL(clicked()), this, SLOT(upJ5Clicked()));
	connect(ui_->btnUpJ6, SIGNAL(clicked()), this, SLOT(upJ6Clicked()));
	connect(ui_->btnDownJ1, SIGNAL(clicked()), this, SLOT(downJ1Clicked()));
	connect(ui_->btnDownJ2, SIGNAL(clicked()), this, SLOT(downJ2Clicked()));
	connect(ui_->btnDownJ3, SIGNAL(clicked()), this, SLOT(downJ3Clicked()));
	connect(ui_->btnDownJ4, SIGNAL(clicked()), this, SLOT(downJ4Clicked()));
	connect(ui_->btnDownJ5, SIGNAL(clicked()), this, SLOT(downJ5Clicked()));
	connect(ui_->btnDownJ6, SIGNAL(clicked()), this, SLOT(downJ6Clicked()));
	connect(ui_->btn_jog_enable, SIGNAL(clicked()), this, SLOT(jog_enableClicked()));
*/
}

CobotJogTeachWidget::~CobotJogTeachWidget() {
	ROS_INFO("CobotJogTeachWidget::~CobotJogTeachWidget()");
  delete ui_;
}

void CobotJogTeachWidget::enable() {
  // activate the frame
  parentWidget()->show();
  show();
}

void CobotJogTeachWidget::disable() {
  parentWidget()->hide();
  hide();
}

/*
void CobotJogTeachWidget::resizeEvent(QResizeEvent *event) {
	QSize size = event->size();
	QRect rect(0, 0, size.width(), size.height());
	ui_->gridLayout->setGeometry(rect);
	QRect tmp = ui_->gridLayout->geometry();
//	ROS_INFO("CobotJogTeachWidget::resizeEvent : [%d, %d][%d, %d, %d, %d]", size.width(), size.height(), tmp.x(), tmp.y(), tmp.width(), tmp.height());
}
*/

void CobotJogTeachWidget::updateJointStateUI() {
//	ROS_INFO("CobotJogTeachWidget::updateJointStateUI");
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
		ui_->lineEdit_rx->setText(QString::number(rpy[0], 'f', 4));
		ui_->lineEdit_ry->setText(QString::number(rpy[1], 'f', 4));
		ui_->lineEdit_rz->setText(QString::number(rpy[2], 'f', 4));
	}
}

geometry_msgs::Pose CobotJogTeachWidget::jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error) {
	moveit_msgs::GetPositionFK msg;
	msg.request.header.stamp = ros::Time::now();
	msg.request.fk_link_names = {"tool0"};
	msg.request.robot_state.joint_state = _js;
	error = "OK";
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
