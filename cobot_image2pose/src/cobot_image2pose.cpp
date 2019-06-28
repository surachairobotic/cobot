#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include "cobot_pick/CobotFindObjectAction.h"
#include "cobot_pick/CobotFindObjectGoal.h"
#include "cobot_msgs/PickPlacePoseArray.h"
#include "cobot_msgs/PickPlacePose.h"

int objStatus, basStatus;
cobot_msgs::PickPlacePose stub;
cobot_msgs::PickPlacePoseArray pppa;
typedef actionlib::SimpleActionClient<cobot_pick::CobotFindObjectAction> CobotAction;

void doneObj(const actionlib::SimpleClientGoalState& state, const cobot_pick::CobotFindObjectResultConstPtr& result);
void activeObj();
void feedbackObj(const cobot_pick::CobotFindObjectFeedbackConstPtr& feedback);
void doneBas(const actionlib::SimpleClientGoalState& state, const cobot_pick::CobotFindObjectResultConstPtr& result);
void activeBas();
void feedbackBas(const cobot_pick::CobotFindObjectFeedbackConstPtr& feedback);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_image2pose");
  ros::NodeHandle n;

  try{
    objStatus = basStatus = 0;
    CobotAction ObjectAction("/cobot/find_object", true);
    CobotAction BasketAction("/cobot/find_basket", true);

    cobot_pick::CobotFindObjectGoal action_goal1;
    cobot_pick::CobotFindObjectGoal action_goal2;

    ros::Publisher pub = n.advertise<cobot_msgs::PickPlacePoseArray>("cobot/image2pose/pickplace_array", 1000);
    stub.b_pick = false;
    stub.b_place = false;
    stub.label = "M1";
    pppa.data.push_back(stub);
    stub.label = "M2";
    pppa.data.push_back(stub);
    stub.label = "M3";
    pppa.data.push_back(stub);
    pppa.header.stamp = ros::Time::now();

    ros::Rate loop_rate(10);
    while (ros::ok()) {
      if(!ObjectAction.isServerConnected())
        ObjectAction.waitForServer();
      if(!BasketAction.isServerConnected())
        BasketAction.waitForServer();

      if(objStatus == 0) {
        ObjectAction.sendGoal(action_goal1, &doneObj, &activeObj, &feedbackObj);
        objStatus = 1;
      }
      if(basStatus == 0) {
        BasketAction.sendGoal(action_goal2, &doneBas, &activeBas, &feedbackBas);
        basStatus = 1;
      }

      pppa.header.stamp = ros::Time::now();
      for(int k=0; k<pppa.data.size(); k++) {
        if( (pppa.header.stamp-pppa.data[k].pick.header.stamp).toSec() > 2.5 )
          pppa.data[k].b_pick = false;
        if( (pppa.header.stamp-pppa.data[k].place.header.stamp).toSec() > 2.5 )
          pppa.data[k].b_place = false;
      }
      pub.publish(pppa);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch(int err){
  }
  catch(const std::string &err){
  }

  return 0;
}

void doneObj(const actionlib::SimpleClientGoalState& state, const cobot_pick::CobotFindObjectResultConstPtr& result) {
  ROS_INFO("doneObj");
  if(objStatus == 2) {
    int chk = 0;
    for(int i=0; i<result->labels.size(); i++) {
      for(int j=0; j<pppa.data.size(); j++) {
        if(result->labels[i] == pppa.data[j].label) {
          pppa.data[j].pick.pose = result->poses[i];
          pppa.data[j].pick.header.stamp = ros::Time::now();
          pppa.data[j].b_pick = true;
          chk++;
        }
      }
    }
    if(chk != result->labels.size())
      ROS_WARN("doneObj : chk != result->labels.size()");
    objStatus = 0;
  }
}
void activeObj() {
  ROS_INFO("activeObj");
  if(objStatus == 1)
    objStatus = 2;
}
void feedbackObj(const cobot_pick::CobotFindObjectFeedbackConstPtr& feedback) {
  ROS_INFO("feedbackObj");
}

void doneBas(const actionlib::SimpleClientGoalState& state, const cobot_pick::CobotFindObjectResultConstPtr& result) {
  ROS_INFO("doneBas");
  if(basStatus == 2) {
    int chk = 0;
    for(int i=0; i<result->labels.size(); i++) {
      for(int j=0; j<pppa.data.size(); j++) {
        if(result->labels[i] == pppa.data[j].label) {
          pppa.data[j].place.pose = result->poses[i];
          pppa.data[j].place.header.stamp = ros::Time::now();
          pppa.data[j].b_place = true;
          chk++;
        }
      }
    }
    if(chk != result->labels.size())
      ROS_WARN("doneObj : chk != result->labels.size()");
    basStatus = 0;
  }
}
void activeBas() {
  ROS_INFO("activeBas");
  if(basStatus == 1)
    basStatus = 2;
}
void feedbackBas(const cobot_pick::CobotFindObjectFeedbackConstPtr& feedback) {
  ROS_INFO("feedbackBas");
}
