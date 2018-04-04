#include <MyFrame.h>
#include "ui_multimovedisplay.h"

using namespace my_plugin;

// minimum delay between calls to callback function
const ros::Duration MyFrame::min_delay_(0.5);

MyFrame::MyFrame( MyDisplay* pdisplay, 
									rviz::DisplayContext* context,
                  QWidget* parent)
//									const std::string& robot_description )
  : QWidget(parent)
	, planning_display_(pdisplay)
  , context_(context)
  , ui_(new Ui::MultiMoveDisplayUI)
//  , rm_loader_("robot_description")
{
	ROS_WARN("Frame");
  ui_->setupUi(this);

  connect(ui_->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
  ui_->tabWidget->setCurrentIndex(0);

  connect(ui_->pushButton_step_01, SIGNAL(clicked()), this, SLOT(step_01_Clicked()));

  // start publishing timer.
  init_time_ = ros::Time::now();
  last_callback_time_ = init_time_;
  average_callback_duration_ = min_delay_;
  schedule_request_count_ = 0;
//  publish_timer_ = nh_.createTimer( average_callback_duration_, 
//																		&MyFrame::updateCallback, this, true);

	if(planning_display_) {
		ROS_ERROR("planning_display_ in MyFrame");
//		if(planning_display_.irobot_start_state)
			ROS_ERROR("planning_display_->irobot_start_state in MyFrame");	
	}

//	robot_pose = planning_display_->irobot_start_state->getPose();

/*
	connect(this, 
					SIGNAL(requestNewLabel(const std::vector<geometry_msgs::Pose>)), 
					this,
					SLOT(updateTextbox(const std::vector<geometry_msgs::Pose>) ));
*/
}

MyFrame::~MyFrame()
{
  delete ui_;
}

void MyFrame::tabChanged(int index)
{
	ROS_INFO("tabChanged : index = %d", index);
//  if (scene_marker_ && ui_->tabWidget->tabText(index).toStdString() != TAB_OBJECTS)
//    scene_marker_.reset();
//  else if (ui_->tabWidget->tabText(index).toStdString() == TAB_OBJECTS)
//    selectedCollisionObjectChanged();
	;
}

void MyFrame::step_01_Clicked()
{
	ROS_INFO("step_01_Clicked !!!");
	ui_->plainTextEdit_info->setPlainText("");
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
  move_group_.reset();
  parentWidget()->hide();
  hide();
}

