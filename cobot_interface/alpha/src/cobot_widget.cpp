#include <cobot_widget.h>
#include <cobot_display.h>
#include "cobot_label.h"

#include "ui_cobot_interface_alpha.h"

using namespace cobot_interface;

CobotWidget::CobotWidget( CobotDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent)
	: planning_display_(pdisplay)
  , context_(context)
  , QWidget(parent)
  , ui_(new Ui::CobotInterfaceAlphaUI)
//  , cobot_label_teach_header(ui_->label_teach_header)
{
  ui_->setupUi(this);



//  srv_teach_enable = n.serviceClient<cobot_msgs::EnableNode>("/cobot/cobot_teach");
//  CobotLabel cobot_label_teach_header;
//  connect(cobot_label_teach_header, SIGNAL(clicked()), this, SLOT(enableNodeClicked()));

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
}

CobotWidget::~CobotWidget() {
	ROS_INFO("CobotWidget::~CobotWidget()");
  delete ui_;
}

void CobotWidget::enable() {
  // activate the frame
  parentWidget()->show();
  show();
}

void CobotWidget::disable() {
  parentWidget()->hide();
  hide();
}
