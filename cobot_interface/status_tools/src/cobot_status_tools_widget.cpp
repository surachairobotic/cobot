#include <cobot_status_tools_widget.h>
#include <cobot_status_tools_display.h>

#include <QResizeEvent>
#include <QSize>
#include <QRect>

#include "ui_cobot_status_tools.h"

using namespace cobot_interface;

CobotStatusToolsWidget::CobotStatusToolsWidget( CobotStatusToolsDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent)
	: planning_display_(pdisplay)
  , context_(context)
  , QWidget(parent)
  , ui_(new Ui::CobotToolsUI)
//  , cobot_label_teach_header(ui_->label_teach_header)
{
  ui_->setupUi(this);

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

/*
void CobotStatusToolsWidget::updateJointStateUI() {
	ui_->lineEdit_j1->setText(QString::number(js.position[0], 'f', 4));
	ui_->lineEdit_j2->setText(QString::number(js.position[1], 'f', 4));
	ui_->lineEdit_j3->setText(QString::number(js.position[2], 'f', 4));
	ui_->lineEdit_j4->setText(QString::number(js.position[3], 'f', 4));
	ui_->lineEdit_j5->setText(QString::number(js.position[4], 'f', 4));
	ui_->lineEdit_j6->setText(QString::number(js.position[5], 'f', 4));
}
*/
