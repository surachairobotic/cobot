#include <MyFrame.h>
#include <MyDisplay.h>
#include <QtMath>

#include "ui_multimovedisplay.h"

using namespace my_plugin;

// minimum delay between calls to callback function
const ros::Duration MyFrame::min_delay_(0.5);

MyFrame::MyFrame( MyDisplay* pdisplay, 
									rviz::DisplayContext* context,
                  QWidget* parent)
  : QWidget(parent)
	, planning_display_(pdisplay)
  , context_(context)
  , ui_(new Ui::MultiMoveDisplayUI)
	, move_group_(new moveit::planning_interface::MoveGroupInterface("arm"))
	, joint_state_pub(nh_.advertise<sensor_msgs::JointState>("/cobot/goal", 1))

/*	, joint_state_pub(nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1))
*/
	, joint_debug_publisher_(nh_.advertise<sensor_msgs::JointState>("/my_ui/joint_solution", 1))
{
	ROS_WARN("Frame");
  ui_->setupUi(this);

  connect(ui_->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
  ui_->tabWidget->setCurrentIndex(0);

  connect(ui_->pushButton_step_01, SIGNAL(clicked()), this, SLOT(step_01_Clicked()));

  connect(ui_->plan_button, SIGNAL(clicked()), this, SLOT(planButtonClicked()));
  connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(executeButtonClicked()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));

/*  connect(ui_->lineEdit_j1, SIGNAL(textChanged(const QString &)), this, SLOT(linej1Changed(const QString &)));
  connect(ui_->lineEdit_j1, SIGNAL(&QLineEdit::textEdited), this, SLOT(&MyFrame::linej1Changed));
*/

  connect(ui_->btnDownJ1, SIGNAL(clicked()), this, SLOT(downJ1Clicked()));
  connect(ui_->btnDownJ2, SIGNAL(clicked()), this, SLOT(downJ2Clicked()));
  connect(ui_->btnDownJ3, SIGNAL(clicked()), this, SLOT(downJ3Clicked()));
  connect(ui_->btnDownJ4, SIGNAL(clicked()), this, SLOT(downJ4Clicked()));
  connect(ui_->btnDownJ5, SIGNAL(clicked()), this, SLOT(downJ5Clicked()));
  connect(ui_->btnDownJ6, SIGNAL(clicked()), this, SLOT(downJ6Clicked()));
  connect(ui_->btnUpJ1, SIGNAL(clicked()), this, SLOT(upJ1Clicked()));
  connect(ui_->btnUpJ2, SIGNAL(clicked()), this, SLOT(upJ2Clicked()));
  connect(ui_->btnUpJ3, SIGNAL(clicked()), this, SLOT(upJ3Clicked()));
  connect(ui_->btnUpJ4, SIGNAL(clicked()), this, SLOT(upJ4Clicked()));
  connect(ui_->btnUpJ5, SIGNAL(clicked()), this, SLOT(upJ5Clicked()));
  connect(ui_->btnUpJ6, SIGNAL(clicked()), this, SLOT(upJ6Clicked()));
  connect(ui_->btnDownX, SIGNAL(clicked()), this, SLOT(downXClicked()));
  connect(ui_->btnDownY, SIGNAL(clicked()), this, SLOT(downYClicked()));
  connect(ui_->btnDownZ, SIGNAL(clicked()), this, SLOT(downZClicked()));
  connect(ui_->btnDownRX, SIGNAL(clicked()), this, SLOT(downRXClicked()));
  connect(ui_->btnDownRY, SIGNAL(clicked()), this, SLOT(downRYClicked()));
  connect(ui_->btnDownRZ, SIGNAL(clicked()), this, SLOT(downRZClicked()));
  connect(ui_->btnUpX, SIGNAL(clicked()), this, SLOT(upXClicked()));
  connect(ui_->btnUpY, SIGNAL(clicked()), this, SLOT(upYClicked()));
  connect(ui_->btnUpZ, SIGNAL(clicked()), this, SLOT(upZClicked()));
  connect(ui_->btnUpRX, SIGNAL(clicked()), this, SLOT(upRXClicked()));
  connect(ui_->btnUpRY, SIGNAL(clicked()), this, SLOT(upRYClicked()));
  connect(ui_->btnUpRZ, SIGNAL(clicked()), this, SLOT(upRZClicked()));

  connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(pushButtonClicked()));	
  connect(ui_->pushButton_2, SIGNAL(clicked()), this, SLOT(pushButton_2Clicked()));
  connect(ui_->btn_input, SIGNAL(clicked()), this, SLOT(btn_inputClicked()));

	pub_plan_1 = nh_.advertise<moveit_msgs::DisplayTrajectory>("/my_ui/display_planned_path_1", 1, true);
	pub_plan_2 = nh_.advertise<moveit_msgs::DisplayTrajectory>("/my_ui/display_planned_path_2", 1, true);
	pub_plan_3 = nh_.advertise<moveit_msgs::DisplayTrajectory>("/my_ui/display_planned_path_3", 1, true);

  init_time_ = ros::Time::now();
  last_callback_time_ = init_time_;
  average_callback_duration_ = min_delay_;
  schedule_request_count_ = 0;

//  robotCurrentState.reset(new robot_state::RobotState(robot_model_));
//  if (!robotCurrentState)
//    ROS_ERROR("Could not get RobotState from Model");

	ui_->lineEdit_x->setAlignment(Qt::AlignRight);
	ui_->lineEdit_y->setAlignment(Qt::AlignRight);
	ui_->lineEdit_z->setAlignment(Qt::AlignRight);
	ui_->lineEdit_rx->setAlignment(Qt::AlignRight);
	ui_->lineEdit_ry->setAlignment(Qt::AlignRight);
	ui_->lineEdit_rz->setAlignment(Qt::AlignRight);
	ui_->lineEdit_rw->setAlignment(Qt::AlignRight);

	ui_->lineEdit_j1_x->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j1_y->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j1_z->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j1_rx->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j1_ry->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j1_rz->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j1_rw->setAlignment(Qt::AlignRight);

	ui_->lineEdit_j2_x->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j2_y->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j2_z->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j2_rx->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j2_ry->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j2_rz->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j2_rw->setAlignment(Qt::AlignRight);

	ui_->lineEdit_j3_x->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j3_y->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j3_z->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j3_rx->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j3_ry->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j3_rz->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j3_rw->setAlignment(Qt::AlignRight);

	ui_->lineEdit_j4_x->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j4_y->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j4_z->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j4_rx->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j4_ry->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j4_rz->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j4_rw->setAlignment(Qt::AlignRight);

	ui_->lineEdit_j5_x->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j5_y->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j5_z->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j5_rx->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j5_ry->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j5_rz->setAlignment(Qt::AlignRight);
	ui_->lineEdit_j5_rw->setAlignment(Qt::AlignRight);

	ui_textbox.push_back(ui_->lineEdit_j1);
	ui_textbox.push_back(ui_->lineEdit_j2);
	ui_textbox.push_back(ui_->lineEdit_j3);
	ui_textbox.push_back(ui_->lineEdit_j4);
	ui_textbox.push_back(ui_->lineEdit_j5);
	ui_textbox.push_back(ui_->lineEdit_j6);

	std::vector<double> x;
	x.push_back(-1.442625345);
	x.push_back(0.3745256647);
	x.push_back(-0.1796085105);
	x.push_back(0.03117646285);
	x.push_back(-0.1955543147);
	x.push_back(-1.494953367);
	test_point.push_back(x);
	x.clear();
	x.push_back(-1.435452402);
	x.push_back(0.1611274743);
	x.push_back(0.1528505229);
	x.push_back(0.02793424244);
	x.push_back(-0.7236728467);
	x.push_back(-1.487051551);
	test_point.push_back(x);
	x.clear();
	x.push_back(-1.324353151);
	x.push_back(0.1715801926);
	x.push_back(0.1506848699);
	x.push_back(-0.290041204);
	x.push_back(-0.6945478264);
	x.push_back(-1.329470298);
	test_point.push_back(x);
	x.clear();
	x.push_back(-1.186389787);
	x.push_back(0.2316551569);
	x.push_back(0.1305305268);
	x.push_back(-0.6393873235);
	x.push_back(-0.5456751197);
	x.push_back(-1.130911299);
	test_point.push_back(x);
	x.clear();				
	index = 0;
}

MyFrame::~MyFrame()
{
	ROS_INFO("MyFrame::~MyFrame()");
	//planning_display_->joint_state_subscriber_.shutdown();

  delete ui_;
	ROS_INFO("MyFrame::~MyFrame() end");
}

void MyFrame::tabChanged(int index)
{
	QString currentTabText = ui_->tabWidget->tabText(index);
	ROS_INFO("tabChanged : index = %s", currentTabText.toStdString().c_str());
}

void MyFrame::step_01_Clicked()
{
	ROS_INFO("step_01_Clicked !!!");
	ui_->plainTextEdit_info->setPlainText("");
}

void MyFrame::changePlanningGroupHelper()
{
	;
//  if (!planning_display_->getPlanningSceneMonitor())
//    return;

//  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::fillStateSelectionOptions, this));
//  planning_display_->addMainLoopJob(
//      boost::bind(&MotionPlanningFrame::populateConstraintsList, this, std::vector<std::string>()));

/*  const robot_model::RobotModelConstPtr& kmodel = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();
  planning_display_->addMainLoopJob(
      boost::bind(&MotionPlanningParamWidget::setGroupName, ui_->planner_param_treeview, group));

  if (!group.empty() && kmodel)
  {
    if (move_group_ && move_group_->getName() == group)
      return;
    ROS_INFO("Constructing new MoveGroup connection for group '%s' in namespace '%s'", group.c_str(),
             planning_display_->getMoveGroupNS().c_str());
    moveit::planning_interface::MoveGroupInterface::Options opt(group);
    opt.robot_model_ = kmodel;
    opt.robot_description_.clear();
    opt.node_handle_ = ros::NodeHandle(planning_display_->getMoveGroupNS());
    try
    {
      move_group_.reset(new moveit::planning_interface::MoveGroupInterface(
          opt, context_->getFrameManager()->getTFClientPtr(), ros::WallDuration(30, 0)));
      if (planning_scene_storage_)
        move_group_->setConstraintsDatabase(ui_->database_host->text().toStdString(), ui_->database_port->value());
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    planning_display_->addMainLoopJob(
        boost::bind(&MotionPlanningParamWidget::setMoveGroup, ui_->planner_param_treeview, move_group_));
    if (move_group_)
    {
      move_group_->allowLooking(ui_->allow_looking->isChecked());
      move_group_->allowReplanning(ui_->allow_replanning->isChecked());
      moveit_msgs::PlannerInterfaceDescription desc;
      if (move_group_->getInterfaceDescription(desc))
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlannersList, this, desc));
      planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this),
                                          "populateConstraintsList");

      if (first_time_)
      {
        first_time_ = false;
        const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
        if (ps)
        {
          planning_display_->setQueryStartState(ps->getCurrentState());
          planning_display_->setQueryGoalState(ps->getCurrentState());
        }
        // This ensures saved UI settings applied after planning_display_ is ready
        planning_display_->useApproximateIK(ui_->approximate_ik->isChecked());
      }
    }
  }
*/
}

void MyFrame::updateTextbox(const std::vector<geometry_msgs::Pose> image)
{
	ROS_INFO("Q_SLOT : updateTextbox !!!");
/*	if(ui_)
		ui_->plainTextEdit_info->setPlainText("Q_SLOT : updateTextbox !!!");
*/
}

// set the callback timer to fire if needed.
// Return true if callback should happen immediately
bool MyFrame::setCallbackTimer(bool new_update_request)
{
  publish_timer_.stop();

  const ros::Time now = ros::Time::now();
  const ros::Duration desired_delay = std::max(min_delay_, average_callback_duration_ * 1.2);
  ros::Duration sec_since_last_callback = now - last_callback_time_;
  ros::Duration sec_til_next_callback = desired_delay - sec_since_last_callback;

  if (schedule_request_count_)
  {
    // need a callback desired_delay seconds after previous callback
    schedule_request_count_ += new_update_request ? 1 : 0;
    if (sec_til_next_callback <= ros::Duration(0.0001))
    {
      // just run the callback now
      return true;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (new_update_request)
  {
    if (sec_til_next_callback < min_delay_)
    {
      // been a while.  Use min_delay_.
      // Set last_callback_time_ to prevent firing too early
      sec_til_next_callback = min_delay_;
      sec_since_last_callback = desired_delay - sec_til_next_callback;
      last_callback_time_ = now - sec_since_last_callback;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (!init_time_.isZero())
  {
    // for the first few seconds after startup call the callback periodically
    // to ensure rviz gets the initial state.
    // Without this rviz does not show some state until markers are moved.
    if ((now - init_time_).sec >= 8)
    {
      init_time_ = ros::Time(0, 0);
      return false;
    }
    else
    {
      publish_timer_.setPeriod(std::max(ros::Duration(1.0), average_callback_duration_ * 2));
      publish_timer_.start();
      return false;
    }
  }
  else
  {
    // nothing to do.  No callback needed.
    return false;
  }
}

// Indicate that the world or the robot has changed and
// the new state needs to be updated and published to rviz
void MyFrame::scheduleUpdate()
{
  // schedule an update callback for the future.
  // If the callback should run now, call it.
  if (setCallbackTimer(true))
    updateCallback(ros::TimerEvent());
}

/* callback called when it is time to publish */
void MyFrame::updateCallback(const ros::TimerEvent& e)
{
  ros::Time tbegin = ros::Time::now();
  publish_timer_.stop();

  // do the actual calculations and publishing
  updateAll();

  // measure time spent in callback for rate limiting
  ros::Time tend = ros::Time::now();
  average_callback_duration_ = (average_callback_duration_ + (tend - tbegin)) * 0.5;
  last_callback_time_ = tend;
  schedule_request_count_ = 0;

  // schedule another callback if needed
  setCallbackTimer(true);
}

/* Calculate new positions and publish results to rviz */
void MyFrame::updateAll()
{
	ROS_INFO("updateTimer...");		
	

/*	if (frame_)	{
		for(int i=0; i<robot_pose.size(); i++) {
			debug_message = debug_message + QString::fromStdString("[");
			debug_message = debug_message + QString::number(i);
			debug_message = debug_message + QString::fromStdString("]");
			debug_message = debug_message + QString::fromStdString(PoseString(robot_pose[i]));
			debug_message = debug_message + QString::fromStdString("\r\n");
		}
		frame_->ui_->plainTextEdit_info->setPlainText(debug_message);	
	}
*/
}

/*
///////////////////////////////////////////////////////////////////////
cControl::cControl(const std::string& _group_name, const std::string& _end_effector_name)
    :spinner(1), 
		group_name(_group_name), 
		end_effector_name(_end_effector_name),
		move_group(group_name), 
		b_start_sub(false), 
		robot_model_loader("robot_description"),
		joint_model_group(NULL)

void cControl::init(){
  spinner.start();
  move_group.setEndEffectorLink(end_effector_name);
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  joint_model_group = kinematic_model->getJointModelGroup (group_name);


  ros::NodeHandle n;
  pub_goal = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/goal", 1000);
}

///////////////////////////////////////////////////////////////////////
void MotionPlanningFrame::changePlanningGroupHelper()
{
  const robot_model::RobotModelConstPtr& kmodel = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();

  if (!group.empty() && kmodel)
  {
    if (move_group_ && move_group_->getName() == group)
      return;
    ROS_INFO("Constructing new MoveGroup connection for group '%s' in namespace '%s'", group.c_str(),
             planning_display_->getMoveGroupNS().c_str());
    moveit::planning_interface::MoveGroupInterface::Options opt(group);
    opt.robot_model_ = kmodel;
    opt.robot_description_.clear();
    opt.node_handle_ = ros::NodeHandle(planning_display_->getMoveGroupNS());
    try
    {
      move_group_.reset(new moveit::planning_interface::MoveGroupInterface(
          opt, context_->getFrameManager()->getTFClientPtr(), ros::WallDuration(30, 0)));
      if (planning_scene_storage_)
        move_group_->setConstraintsDatabase(ui_->database_host->text().toStdString(), ui_->database_port->value());
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    planning_display_->addMainLoopJob(
        boost::bind(&MotionPlanningParamWidget::setMoveGroup, ui_->planner_param_treeview, move_group_));
    if (move_group_)
    {
      move_group_->allowLooking(ui_->allow_looking->isChecked());
      move_group_->allowReplanning(ui_->allow_replanning->isChecked());
      moveit_msgs::PlannerInterfaceDescription desc;
      if (move_group_->getInterfaceDescription(desc))
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlannersList, this, desc));
      planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this),
                                          "populateConstraintsList");

      if (first_time_)
      {
        first_time_ = false;
        const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
        if (ps)
        {
          planning_display_->setQueryStartState(ps->getCurrentState());
          planning_display_->setQueryGoalState(ps->getCurrentState());
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////
*/

void MyFrame::enable()
{
//  ui_->planning_algorithm_combo_box->clear();
//  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
//  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
//  ui_->object_status->setText("");

  // activate the frame
  parentWidget()->show();
  show();
}

void MyFrame::disable()
{
	ROS_INFO("MyFrame::disable()");
//  move_group_.reset();
  parentWidget()->hide();
  hide();
}

void MyFrame::pubCurrentRobotState(std::vector<double>& vPosition)
{
//	pubUiSeed(vPosition);
	ROS_INFO("MyFrame::pubCurrentRobotState !!!");

	sensor_msgs::JointState msgJoint;
	msgJoint.name = planning_display_->sJointsName;
	msgJoint.position = vPosition;
	std::vector<double> velocity;
	for(int i=0; i<vPosition.size(); i++)	velocity.push_back(0.7);
	msgJoint.velocity = velocity;
  joint_state_pub.publish(msgJoint);
	ROS_INFO("MyFrame::pubCurrentRobotState-END !!!");
}

void MyFrame::pubUiSeed(const std::vector<double>& vPosition)
{
	ROS_INFO("MyFrame::pubUiSeed !!!");
//	for(int i=0; i<vPosition.size(); i++)
//		ui_textbox[i]->setText(QString::number(qRadiansToDegrees(vPosition[i]), 'f', 4));
	for(int i=0; i<vPosition.size(); i++)
		ui_textbox[i]->setText(QString::number(vPosition[i], 'f', 4));
	ROS_INFO("MyFrame::pubUiSeed-A !!!");

	std::vector<geometry_msgs::Pose> pose_fk;
	std::vector<std::string> link_names;
	link_names.push_back(planning_display_->end_link);
	ROS_INFO("MyFrame::pubUiSeed-B !!!");
	planning_display_->solver->getPositionFK(link_names, vPosition, pose_fk);
	ROS_INFO("MyFrame::pubUiSeed-C !!!");
	geometry_msgs::Pose pose = pose_fk[0];

	ui_->lineEdit_eef_x->setText(QString::number(pose.position.x, 'f', 4));
	ui_->lineEdit_eef_y->setText(QString::number(pose.position.y, 'f', 4));
	ui_->lineEdit_eef_z->setText(QString::number(pose.position.z, 'f', 4));
	ROS_INFO("ui_->lineEdit");

	tf::Quaternion q_ori;
	quaternionMsgToTF(pose.orientation , q_ori);
	ROS_INFO("orientation : %lf, %lf, %lf, %lf", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 m(q_ori);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
//	double angle = qRadiansToDegrees(pose.orientation.x);
	ui_->lineEdit_eef_rx->setText(QString::number(roll, 'f', 4));
//	angle = qRadiansToDegrees(pose.orientation.y);
	ui_->lineEdit_eef_ry->setText(QString::number(pitch, 'f', 4));
//	angle = qRadiansToDegrees(pose.orientation.z);
	ui_->lineEdit_eef_rz->setText(QString::number(yaw, 'f', 4));
	ROS_INFO("MyFrame::pubUiSeed-END !!!");
}
