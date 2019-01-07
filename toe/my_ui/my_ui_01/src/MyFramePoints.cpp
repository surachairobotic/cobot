#include <MyFrame.h>
#include <MyDisplay.h>
#include <QString>


#include "ui_multimovedisplay.h"

using namespace my_plugin;

// Points tab

void MyFrame::btn_add_pointsClicked()
{
//  ROS_INFO("MyFrame::btn_add_pointsClicked()");
	std::vector<double> ik_seed_state;
	planning_display_->robotCurrentState->copyJointGroupPositions(planning_display_->planning_group, ik_seed_state);
	geometry_msgs::Pose pose = getEEFpose(planning_display_->robotCurrentState);
	ROS_INFO("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", ik_seed_state[0], ik_seed_state[1], ik_seed_state[2], ik_seed_state[3], ik_seed_state[4], ik_seed_state[5], pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	
	std::string str = std::to_string(ik_seed_state[0]);
	for(int i=1; i<ik_seed_state.size(); i++)
	  str += (',' + std::to_string(ik_seed_state[i]));
	str += (',' + std::to_string(pose.position.x));
	str += (',' + std::to_string(pose.position.y));
	str += (',' + std::to_string(pose.position.z));
	str += (',' + std::to_string(pose.orientation.x));
	str += (',' + std::to_string(pose.orientation.y));
	str += (',' + std::to_string(pose.orientation.z));
	str += (',' + std::to_string(pose.orientation.w));
	
  QString qstr = QString::fromStdString(str);
	ui_->list_points->addItem(qstr);
	ROS_INFO("count = %d", ui_->list_points->count());
}

void MyFrame::btn_del_pointsClicked()
{
  delete ui_->list_points->takeItem(ui_->list_points->currentRow());
	ROS_INFO("count = %d", ui_->list_points->count());
}

void MyFrame::btn_save_pointsClicked()
{
  ROS_INFO("MyFrame::btn_save_pointsClicked()");
  std::string dir = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/";
  FILE *fp = fopen( (dir + "point_cloud.txt").c_str() , "wt");
  if( !fp ) {
    ROS_ERROR("Cannot open point_cloud.txt");
    return ;
  }
  ROS_INFO("File is ok.");
//  QList<QListWidgetItem *> qlist = ui_->list_points->selectedItems();
  ROS_INFO("qlist.size = %d", ui_->list_points->count());
  for(int i=0; i<ui_->list_points->count(); i++) {
    fprintf(fp, "%s\r\n", ui_->list_points->item(i)->text().toStdString().c_str() );
    ROS_INFO("%s", ui_->list_points->item(i)->text().toStdString().c_str() );
  }
  if( fp )
    fclose(fp);
  ROS_INFO("close file.");
}

