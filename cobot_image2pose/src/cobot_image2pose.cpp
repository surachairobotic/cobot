#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <math.h>
#include "cobot_pick/CobotFindObjectAction.h"
#include "cobot_pick/CobotFindObjectGoal.h"
#include "cobot_msgs/PickPlacePoseArray.h"
#include "cobot_msgs/PickPlacePose.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>


#include <typeinfo>

bool en = false;
int objStatus, basStatus;
cobot_msgs::PickPlacePose stub;
cobot_msgs::PickPlacePoseArray pppa;
typedef actionlib::SimpleActionClient<cobot_pick::CobotFindObjectAction> CobotAction;
ros::Publisher pub_box_marker, pub_pick;

void doneObj(const actionlib::SimpleClientGoalState& state, const cobot_pick::CobotFindObjectResultConstPtr& result);
void activeObj();
void feedbackObj(const cobot_pick::CobotFindObjectFeedbackConstPtr& feedback);
void doneBas(const actionlib::SimpleClientGoalState& state, const cobot_pick::CobotFindObjectResultConstPtr& result);
void activeBas();
void feedbackBas(const cobot_pick::CobotFindObjectFeedbackConstPtr& feedback);
void callback_en(const std_msgs::Bool& msg);
void callback_cmd(const std_msgs::String& msg);
void pick_marker(const geometry_msgs::Pose& msg, double inx);
geometry_msgs::Pose pose_transform(const geometry_msgs::Pose& msg, double l, double deg);

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
    pub_pick = n.advertise<geometry_msgs::PoseArray>("/cobot/cobot_core/pick", 1000);
    ros::Subscriber sub_en = n.subscribe("/cobot/image2pose/enable", 10, callback_en);
    ros::Subscriber sub_cmd = n.subscribe("/cobot/image2pose/cmd", 10, callback_cmd);
    pub_box_marker = n.advertise<visualization_msgs::Marker>("/cobot/marker/box", 10);

    stub.b_pick = -1;
    stub.b_place = -1;
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

      if(en) {
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
            pppa.data[k].b_pick = -1;
          if( (pppa.header.stamp-pppa.data[k].place.header.stamp).toSec() > 2.5 )
            pppa.data[k].b_place = -1;
        }
        pub.publish(pppa);
        // pick_marker(pppa.data[0].pick.pose, 0);
        // geometry_msgs::Pose xx = pose_transform(pppa.data[0].pick.pose, 0.05, 0.0);
        // pick_marker(xx, 1);

//        geometry_msgs::Pose pose_transform(const geometry_msgs::Pose& msg, double l, double deg) {
      }
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
    for(int j=0; j<pppa.data.size(); j++)
      pppa.data[j].b_pick = -1;
    for(int i=0; i<result->labels.size(); i++) {
      for(int j=0; j<pppa.data.size(); j++) {
        if(result->labels[i] == pppa.data[j].label) {
          pppa.data[j].pick.pose = result->poses[i];
          pppa.data[j].pick.header.stamp = ros::Time::now();
          pppa.data[j].b_pick = i;
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
    for(int j=0; j<pppa.data.size(); j++)
      pppa.data[j].b_place = -1;
    for(int i=0; i<result->labels.size(); i++) {
      for(int j=0; j<pppa.data.size(); j++) {
        if(result->labels[i] == pppa.data[j].label) {
          pppa.data[j].place.pose = result->poses[i];
          pppa.data[j].place.header.stamp = ros::Time::now();
          pppa.data[j].b_place = i;
          chk++;
        }
      }
    }
    if(chk != result->labels.size())
      ;//ROS_WARN("doneObj : chk != result->labels.size()");
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

void callback_en(const std_msgs::Bool& msg) {
  en = msg.data;
}

void callback_cmd(const std_msgs::String& msg) {
  ROS_INFO("callback_cmd : %s", msg.data.c_str());
  for(int i=0; i<pppa.data.size(); i++) {
    ROS_INFO("pppa.data[%d].label : %s", i, pppa.data[i].label.c_str());
    if(msg.data.compare(pppa.data[i].label) == 0) {
      ROS_INFO("msg.data.compare : true");
      if(pppa.data[i].b_pick > -1 && pppa.data[i].b_place > -1) {
        ROS_INFO("pppa.data[i].b_pick && pppa.data[i].b_place : true");
        pick_marker(pppa.data[i].pick.pose, 0);
        geometry_msgs::Pose _pick = pose_transform(pppa.data[i].pick.pose, 0.05, 0.0);
        pick_marker(_pick, 1);
        geometry_msgs::Pose _place = pose_transform(pppa.data[i].place.pose, 0.05, 0.0);
        pick_marker(_place, 2);
        pick_marker(pppa.data[i].place.pose, 3);
        geometry_msgs::PoseArray msg;
        msg.poses.push_back(pose_transform(pppa.data[i].pick.pose, 0.05, 0.0));
        msg.poses.push_back(pppa.data[i].pick.pose);
        pub_pick.publish(msg);
      }
      else
        ROS_INFO("pppa.data[i].b_pick && pppa.data[i].b_place : false");
    }
    else
      ROS_INFO("msg.data.compare : false");
  }
}

geometry_msgs::Pose pose_transform(const geometry_msgs::Pose& msg, double l, double deg) {
  Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(msg.orientation, q);
  Eigen::Matrix3d r_input;

  double k = deg*M_PI/180;
  r_input(0,0) = cos(k);
  r_input(0,1) = -sin(k);
  r_input(0,2) = 0.0;
  r_input(1,0) = sin(k);
  r_input(1,1) = cos(k);
  r_input(1,2) = 0.0;
  r_input(2,0) = 0.0;
  r_input(2,1) = 0.0;
  r_input(2,2) = 1.0;
  Eigen::Matrix3d r = q.normalized().toRotationMatrix() * r_input;

  geometry_msgs::Pose pp = msg;
  pp.position.x = msg.position.x - r(0,2)*l;
  pp.position.y = msg.position.y - r(1,2)*l;
  pp.position.z = msg.position.z - r(2,2)*l;

  return pp;
}

void pick_marker(const geometry_msgs::Pose& msg, double inx) {
  geometry_msgs::Point p1, p2, p3, p4;
  p1 = msg.position;

  Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(msg.orientation, q);
  Eigen::Matrix3d r_input;

  double k = M_PI;
  r_input(0,0) = cos(k);
  r_input(0,1) = -sin(k);
  r_input(0,2) = 0.0;
  r_input(1,0) = sin(k);
  r_input(1,1) = cos(k);
  r_input(1,2) = 0.0;
  r_input(2,0) = 0.0;
  r_input(2,1) = 0.0;
  r_input(2,2) = 1.0;
  Eigen::Matrix3d r = q.normalized().toRotationMatrix() * r_input;

  double scale = 0.005, l = 0.025;
  p2.x = r(0,0)*l + p1.x;
  p2.y = r(1,0)*l + p1.y;
  p2.z = r(2,0)*l + p1.z;

  p3.x = r(0,1)*l + p1.x;
  p3.y = r(1,1)*l + p1.y;
  p3.z = r(2,1)*l + p1.z;

  p4.x = p1.x - r(0,2)*l;
  p4.y = p1.y - r(1,2)*l;
  p4.z = p1.z - r(2,2)*l;

  visualization_msgs::Marker pp1, pp2, pp3;

  pp1.header.frame_id = "world";
  pp1.header.stamp = ros::Time::now();
  pp1.action = pp1.ADD;
  pp1.type = pp1.ARROW;
  pp1.id = 0+(3*inx);

  pp1.points.clear();
  pp1.points.push_back(p2);
  pp1.points.push_back(p1);

  pp1.scale.x = scale;
  pp1.scale.y = scale;
  pp1.scale.z = scale;

  pp1.color.a = 1.0;
  pp1.color.r = 1.0;
  pp1.color.g = 0.0;
  pp1.color.b = 0.0;
  pub_box_marker.publish(pp1);

  pp2.header.frame_id = "world";
  pp2.header.stamp = ros::Time::now();
  pp2.action = pp2.ADD;
  pp2.type = pp2.ARROW;
  pp2.id = 1+(3*inx);

  pp2.points.clear();
  pp2.points.push_back(p3);
  pp2.points.push_back(p1);

  pp2.scale.x = scale;
  pp2.scale.y = scale;
  pp2.scale.z = scale;

  pp2.color.a = 1.0;
  pp2.color.r = 0.0;
  pp2.color.g = 1.0;
  pp2.color.b = 0.0;
  pub_box_marker.publish(pp2);

  pp3.header.frame_id = "world";
  pp3.header.stamp = ros::Time::now();
  pp3.action = pp3.ADD;
  pp3.type = pp3.ARROW;
  pp3.id = 2+(3*inx);

  pp3.points.clear();
  pp3.points.push_back(p4);
  pp3.points.push_back(p1);

  pp3.scale.x = scale;
  pp3.scale.y = scale;
  pp3.scale.z = scale;

  pp3.color.a = 1.0;
  pp3.color.r = 0.0;
  pp3.color.g = 0.0;
  pp3.color.b = 1.0;
  pub_box_marker.publish(pp3);
}

/*
if m == 0:
    poses.poses = [self.p_dot(a, 0.05, 180.0), self.p_dot(a, -0.005, 180.0), self.p_dot(a, 0.05, 180.0), self.p_dot(b, 0.1, 0.0), self.p_dot(b, 0.00, 0.0), self.p_dot(b, 0.1, 0.0)]
elif m == 1:
    poses.poses = [self.p_dot(a, 0.05, 180.0), self.p_dot(a, -0.005, 180.0), self.p_dot(a, 0.05, 180.0), self.p_dot(b, 0.00, 0.0)]
*/
