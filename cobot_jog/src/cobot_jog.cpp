#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "cobot_msgs/Jog.h"
#include "cobot_msgs/EnableNode.h"
#include "cobot_msgs/ChangeMode.h"
#include "moveit_msgs/GetPositionFK.h"
#include "moveit_msgs/GetPositionIK.h"
#include "tf/tf.h"

sensor_msgs::JointState js, goal;
std::string DRIVER_KEY = "";
ros::ServiceClient srv_mode, srv_fk, srv_ik;
ros::Publisher pub_goal;
bool is_enable = false;

void callback_js(const sensor_msgs::JointState &js);
void callback_jog(const cobot_msgs::Jog &msg);
bool handle_enable_node(cobot_msgs::EnableNode::Request  &req, cobot_msgs::EnableNode::Response &res);
geometry_msgs::Pose CobotFK(const sensor_msgs::JointState &_js, std::vector<double> &_pose);
sensor_msgs::JointState CobotIK(const geometry_msgs::Pose &_p, const sensor_msgs::JointState &_in);

int main(int argc, char **argv) {
  ros::init(argc, argv, "cobot_jog");
  ros::NodeHandle n;
  ros::Subscriber sub_js = n.subscribe("/cobot/joint_states", 100, callback_js);
  ros::Subscriber sub_ui = n.subscribe("/cobot/cobot_jog", 100, callback_jog);
  ros::ServiceServer service_enable  = n.advertiseService("/cobot/cobot_jog/enable" , handle_enable_node);
  pub_goal = n.advertise<sensor_msgs::JointState>("/cobot/goal", 100);
  srv_mode = n.serviceClient<cobot_msgs::ChangeMode>("/cobot/cobot_core/change_mode");
  srv_fk = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
  srv_ik = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

  ROS_INFO("COBOT_JOG : START");
  ros::Rate loop_rate(100);
  while(ros::ok()) {
//    ROS_INFO("%d : %s", is_enable, DRIVER_KEY.c_str());
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

    goal.name = js.name;
    goal.velocity = {msg.velocity, msg.velocity, msg.velocity, msg.velocity, msg.velocity, msg.velocity};
    if(msg.cmd == "HOME") {
      goal.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    else if(msg.cmd[0] == 'J') {
      goal.name = {msg.cmd};
      goal.velocity = {msg.velocity};
      int indx = (msg.cmd[1]-'0')-1;
      goal.position = {js.position[indx]+msg.resolution};
    }
    else if(msg.cmd[0] == 'C') {
      std::vector<double> pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      geometry_msgs::Pose pp = CobotFK(js, pose);
      bool orien = false;
      if(msg.cmd[1] == 'X')
        pp.position.x += msg.resolution;
      else if(msg.cmd[1] == 'Y')
        pp.position.y += msg.resolution;
      else if(msg.cmd[1] == 'Z')
        pp.position.z += msg.resolution;
      else if(msg.cmd[1] == 'R') {
        pose[3] += msg.resolution;
        orien = true;
      }
      else if(msg.cmd[1] == 'P') {
        orien = true;
        pose[4] += msg.resolution;
      }
      else if(msg.cmd[1] == 'W') {
        orien = true;
        pose[5] += msg.resolution;
      }

      if(orien) {
        tf::Quaternion q_ori;
      	q_ori.setRPY(pose[3], pose[4], pose[5]);
      	tf::quaternionTFToMsg(q_ori, pp.orientation);
      }

      sensor_msgs::JointState jj = CobotIK(pp, js);
      goal.position = jj.position;
    }
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

geometry_msgs::Pose CobotFK(const sensor_msgs::JointState &_js, std::vector<double> &_pose) {
	moveit_msgs::GetPositionFK msg;
	msg.request.header.stamp = ros::Time::now();
	msg.request.fk_link_names = {"tool0"};
	msg.request.robot_state.joint_state = _js;
	if(srv_fk.call(msg)) {
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
	}
	return msg.response.pose_stamped[0].pose;
}

sensor_msgs::JointState CobotIK(const geometry_msgs::Pose &_p, const sensor_msgs::JointState &_in) {
  moveit_msgs::GetPositionIK msg;
  msg.request.ik_request.group_name = "arm";
  msg.request.ik_request.robot_state.joint_state = _in;
  msg.request.ik_request.ik_link_name = "tool0";
  msg.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  msg.request.ik_request.pose_stamped.pose = _p;
  if(srv_ik.call(msg)) {
    return msg.response.solution.joint_state;
  }
  else {
		ROS_ERROR("Failed to call service compute_ik");
	}
}
