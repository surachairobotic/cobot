#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "cobot_msgs/Jog.h"
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/ChangeMode.h"

sensor_msgs::JointState js, goal;
std::string DRIVER_KEY = "";
ros::ServiceClient srv_mode;
ros::Publisher pub_goal;
bool is_enable = false;

void callback_js(const sensor_msgs::JointState &js);
void callback_jog(const cobot_msgs::Jog &msg);
bool handle_enable_node(cobot_msgs::EnableNode::Request  &req, cobot_msgs::EnableNode::Response &res);

int main(int argc, char **argv) {
  ros::init(argc, argv, "cobot_jog");
  ros::NodeHandle n;
  ros::Subscriber sub_js = n.subscribe("/cobot/joint_states", 100, callback_js);
  ros::Subscriber sub_ui = n.subscribe("/cobot/cobot_jog", 100, callback_jog);
  ros::ServiceServer service_enable  = n.advertiseService("/cobot/cobot_jog/enable" , handle_enable_node);
  pub_goal = n.advertise<sensor_msgs::JointState>("/cobot/goal", 100);
  srv_mode = n.serviceClient<cobot_msgs::ChangeMode>("/cobot/cobot_core/change_mode");

  ROS_INFO("COBOT_JOG : START");
  ros::Rate loop_rate(100);
  while(ros::ok()) {
    ROS_INFO("%d : %s", is_enable, DRIVER_KEY.c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("COBOT_JOG : END");
}

void callback_js(const sensor_msgs::JointState &_js) {
  js = _js;
}

void callback_jog(const cobot_msgs::Jog &msg) {
  ROS_INFO("callback_jog : %s --> %lf", msg.cmd.c_str(), msg.resolution);
  if(is_enable) {
    bool is_correct = true;
    int indx = -1;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = DRIVER_KEY;
    goal.name = {"J1", "J2", "J3", "J4", "J5", "J6"};
    goal.position = js.position;
    goal.velocity = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    if(msg.cmd == "J1")
      goal.position[0] += msg.resolution;
    else if(msg.cmd == "J2")
      goal.position[1] += msg.resolution;
    else if(msg.cmd == "J3")
      goal.position[2] += msg.resolution;
    else if(msg.cmd == "J4")
      goal.position[3] += msg.resolution;
    else if(msg.cmd == "J5")
      goal.position[4] += msg.resolution;
    else if(msg.cmd == "J6")
      goal.position[5] += msg.resolution;
    else if(msg.cmd == "HOME")
      goal.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    pub_goal.publish(goal);
  }
}

bool handle_enable_node(cobot_msgs::EnableNode::Request  &req, cobot_msgs::EnableNode::Response &res) {
  is_enable = false;
  res.error.data = "OK";
  if(req.enable) {
    cobot_msgs::ChangeMode msg;
    msg.request.mode = "JOG_MODE";
    if (srv_mode.call(msg)) {
      if(msg.response.error == "OK") {
        DRIVER_KEY = msg.response.key;
        is_enable = true;
      }
    }
    else {
      ROS_ERROR("Failed to call service change_mode");
      res.error.data = "ERR: Failed to call service enable_node";
    }
  }
  return true;
}
