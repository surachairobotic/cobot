#include <cobot_status_tools_widget.h>
#include <cobot_status_tools_display.h>

#include <QResizeEvent>
#include <QSize>
#include <QRect>
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/GetPositionFK.h"
#include "cobot_msgs/ReadJointStateFile.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "tf/tf.h"

#include "ui_cobot_status_tools.h"

using namespace cobot_interface;

CobotStatusToolsWidget::CobotStatusToolsWidget( CobotStatusToolsDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent)
	: cobot_display_(pdisplay)
  , context_(context)
  , QWidget(parent)
  , ui_(new Ui::CobotToolsUI)
//  , cobot_label_teach_header(ui_->label_teach_header)
{
  ui_->setupUi(this);

	connect(cobot_display_, SIGNAL(jsUpdate()), this, SLOT(updateJointStateUI()));

	connect(ui_->btn_add, SIGNAL(clicked()), this, SLOT(addPointClicked()));
	connect(ui_->btn_del, SIGNAL(clicked()), this, SLOT(delPointClicked()));
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

CobotStatusToolsWidget::~CobotStatusToolsWidget() {
	ROS_INFO("CobotStatusToolsWidget::~CobotStatusToolsWidget()");
  delete ui_;
}

void CobotStatusToolsWidget::enable() {
  // activate the frame
  parentWidget()->show();
  show();
}

void CobotStatusToolsWidget::disable() {
  parentWidget()->hide();
  hide();
}

/*
void CobotStatusToolsWidget::resizeEvent(QResizeEvent *event) {
	QSize size = event->size();
	QRect rect(0, 0, size.width(), size.height());
	ui_->gridLayout->setGeometry(rect);
	QRect tmp = ui_->gridLayout->geometry();
//	ROS_INFO("CobotStatusToolsWidget::resizeEvent : [%d, %d][%d, %d, %d, %d]", size.width(), size.height(), tmp.x(), tmp.y(), tmp.width(), tmp.height());
}
*/

void CobotStatusToolsWidget::updateJointStateUI() {
//	ROS_INFO("CobotStatusToolsWidget::updateJointStateUI");
	ui_->lineEdit_j1->setText(QString::number(cobot_display_->js.position[0], 'f', 4));
	ui_->lineEdit_j2->setText(QString::number(cobot_display_->js.position[1], 'f', 4));
	ui_->lineEdit_j3->setText(QString::number(cobot_display_->js.position[2], 'f', 4));
	ui_->lineEdit_j4->setText(QString::number(cobot_display_->js.position[3], 'f', 4));
	ui_->lineEdit_j5->setText(QString::number(cobot_display_->js.position[4], 'f', 4));
	ui_->lineEdit_j6->setText(QString::number(cobot_display_->js.position[5], 'f', 4));

	double cartisian[6];
	if(jsCartesian(cobot_display_->js, cartisian)) {
		ui_->lineEdit_x->setText(QString::number(cartisian[0], 'f', 4));
		ui_->lineEdit_y->setText(QString::number(cartisian[1], 'f', 4));
		ui_->lineEdit_z->setText(QString::number(cartisian[2], 'f', 4));
		ui_->lineEdit_roll->setText(QString::number(cartisian[3], 'f', 4));
		ui_->lineEdit_pitch->setText(QString::number(cartisian[4], 'f', 4));
		ui_->lineEdit_yaw->setText(QString::number(cartisian[5], 'f', 4));
	}
}

void CobotStatusToolsWidget::addPointClicked() {
	ROS_INFO("CobotStatusToolsWidget::addPointClicked()");
	ROS_INFO("current row : %d", ui_->listWidget_points->currentRow());
	cobot_msgs::EditJointStateFile req_edit_js_file;
	req_edit_js_file.request.index = ui_->listWidget_points->currentRow();
	req_edit_js_file.request.operation = 1;
	if (srv_edit_js_file.call(req_edit_js_file))
		ROS_INFO("srv_edit_js_file.call is true");
	else
		ROS_INFO("srv_edit_js_file.call is false");
	ROS_INFO("req_edit_js_file.response.error.data : %s", req_edit_js_file.response.error.data.c_str());
	if (req_edit_js_file.response.error.data == "OK")
		updatePointsTable();
}
void CobotStatusToolsWidget::delPointClicked() {
	ROS_INFO("CobotStatusToolsWidget::delPointClicked()");
	ROS_INFO("current row : %d", ui_->listWidget_points->currentRow());
	cobot_msgs::EditJointStateFile req_edit_js_file;
	req_edit_js_file.request.index = ui_->listWidget_points->currentRow();
	req_edit_js_file.request.operation = -1;
	if (srv_edit_js_file.call(req_edit_js_file))
		ROS_INFO("srv_edit_js_file.call is true");
	else
		ROS_INFO("srv_edit_js_file.call is false");
	ROS_INFO("req_edit_js_file.response.error.data : %s", req_edit_js_file.response.error.data.c_str());
	if (req_edit_js_file.response.error.data == "OK")
		updatePointsTable();
}

void CobotStatusToolsWidget::updatePointsTable() {
	ROS_INFO("CobotStatusToolsWidget::updatePointsTable()");

	cobot_msgs::ReadJointStateFile req_read_point_file;
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
			double cartisian[6];
			if(jsCartesian(js_points[i], cartisian)) {
				QString str;
				str.sprintf("P%d| X:%.4lf, Y:%.4lf, Z:%.4lf", i, cartisian[0], cartisian[1], cartisian[2]);
				ui_->listWidget_points->addItem(str);
			}
		}
	}
}



bool CobotStatusToolsWidget::jsCartesian(const sensor_msgs::JointState &_js, double* _pose) {
	moveit_msgs::GetPositionFK msg;
	msg.request.header.stamp = ros::Time::now();
	msg.request.fk_link_names = {"tool0"};
	msg.request.robot_state.joint_state = _js;
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
			return false;
	}
	return true;
}
