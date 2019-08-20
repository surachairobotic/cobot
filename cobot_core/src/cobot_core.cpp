#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseArray.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "cobot_msgs/ReadJointStateFile.h"
#include "cobot_msgs/ChangeMode.h"
#include "std_msgs/Bool.h"
#include "cobot_dynamixel_driver/change_key.h"
#include "cobot_planner/CobotPlanning.h"
#include "moveit_msgs/GetPositionFK.h"
#include "moveit_msgs/GetPositionIK.h"
#include <actionlib/client/simple_action_client.h>
#include "cobot_msgs/ExecuteAction.h"
#include "tf/tf.h"

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>

std::string COBOT_MODE = "TEACH_MODE";
std::string DRIVER_KEY = "";
sensor_msgs::JointState js;
std::string ori_file = "/home/mtec/catkin_ws/src/cobot/cobot_core/data/joint_state_file.txt";
std::string tmp_file = "/home/mtec/catkin_ws/src/cobot/cobot_core/data/temp_file.txt";

geometry_msgs::PoseArray pp;
bool b_pick = false;
bool b_place = false;
int state_pp = 0;
ros::Publisher pub_vacuum;
class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };

  ros::NodeHandle *n;
};
ROSNode *rn;

void callback_js(const sensor_msgs::JointState &js);
void callback_pick(const geometry_msgs::PoseArray &p);
void callback_pp_done(const std_msgs::Bool &p);
bool editJointStateFileFunc(cobot_msgs::EditJointStateFile::Request  &req, cobot_msgs::EditJointStateFile::Response &res);
bool readJointStateFileFunc(cobot_msgs::ReadJointStateFile::Request  &req, cobot_msgs::ReadJointStateFile::Response &res);
bool changeModeFunc(cobot_msgs::ChangeMode::Request  &req, cobot_msgs::ChangeMode::Response &res);
int countLine(void);
std::string gen_random(const int len);
geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error);
void getIK(geometry_msgs::Pose &_ps);

int main(int argc, char **argv) {
  ros::init(argc, argv, "cobot_core");
  ros::NodeHandle nn;
  rn = new ROSNode(nn);
  ros::Subscriber sub_js = rn->n->subscribe("/cobot/joint_states", 100, callback_js);
  ros::Subscriber sub_pick = rn->n->subscribe("/cobot/cobot_core/pick", 1000, callback_pick);
  ros::Subscriber sub_pp_done = rn->n->subscribe("/cobot/core_message", 10, callback_pp_done);
  pub_vacuum = rn->n->advertise<std_msgs::Bool>("cobot/vacuum_message", 10);

  ros::ServiceServer service_edit_js_file = rn->n->advertiseService("/cobot/cobot_core/edit_js_file", editJointStateFileFunc);
  ros::ServiceServer service_read_js_file = rn->n->advertiseService("/cobot/cobot_core/read_js_file", readJointStateFileFunc);
  ros::ServiceServer service_change_mode  = rn->n->advertiseService("/cobot/cobot_core/change_mode" , changeModeFunc);

  ROS_INFO("COBOT_CORE : START");
  ros::Rate loop_rate(100);
  while(ros::ok()) {
    if(state_pp == 1 || state_pp == 3 || state_pp == 5) {
      cobot_planner::CobotPlanning req_plan;

      std::vector<double> rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::string error_code = "";
      geometry_msgs::Pose cartesian = jsCartesian(js, rpy, error_code);
      if(error_code == "OK")
        req_plan.request.pose_array.push_back(cartesian);
      else
        ROS_INFO("cartesian pose is false");

      if(state_pp == 1) {
        req_plan.request.pose_array.push_back(pp.poses[0]);
        req_plan.request.pose_array.push_back(pp.poses[1]);
        state_pp = 2;
      }
      else if(state_pp == 3) {
        req_plan.request.pose_array.push_back(pp.poses[0]);
        req_plan.request.pose_array.push_back(pp.poses[2]);
        req_plan.request.pose_array.push_back(pp.poses[3]);
        state_pp = 4;
      }
      else if(state_pp == 5) {
        req_plan.request.pose_array.push_back(pp.poses[2]);
        state_pp = 6;
      }

      req_plan.request.joint_names = {"J1", "J2", "J3", "J4", "J5", "J6"};
      req_plan.request.seed = js;
      req_plan.request.type = "line";
      req_plan.request.max_velocity = 0.05;
      req_plan.request.max_acceleration = M_PI/2.0;
      req_plan.request.step_time = 0.05;

      for(int j=0; j<pp.poses.size(); j++) {
        ROS_INFO("pp.poses[%d] : ", j);
        getIK(pp.poses[j]);
      }

      ros::ServiceClient srv_cobot_planning = rn->n->serviceClient<cobot_planner::CobotPlanning>("/cobot/planning");
      if (srv_cobot_planning.call(req_plan)) {
        ROS_INFO("srv_cobot_planning.call is true");
      }
      else {
        ROS_INFO("srv_cobot_planning.call is false");
        assert(0);
        continue;
      }

      ROS_INFO("req_plan.response.error_code : %d", req_plan.response.error_code);
      if (req_plan.response.error_code == 0) {
        actionlib::SimpleActionClient<cobot_msgs::ExecuteAction> ac("cobot_execute", true);
      	ac.waitForServer(); //will wait for infinite time
      	ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        cobot_msgs::ExecuteGoal goal;
        ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
          ROS_INFO("Action did not finish before the time out.");
      	ROS_INFO("execClicked is done.");
      }
      else {
        ROS_ERROR("Plane error code : %d", req_plan.response.error_code);
        assert(0);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("COBOT_CORE : END");
}

// cobot_msgs::EditJointStateFile
void callback_js(const sensor_msgs::JointState &_js) {
  js = _js;
}
bool editJointStateFileFunc(cobot_msgs::EditJointStateFile::Request  &req, cobot_msgs::EditJointStateFile::Response &res) {
  res.error.data = "OK";
  if( !(req.operation == 1 || req.operation == -1) ) {
    res.error.data = "ERR: Operation not found [ADD = 1, DELETE = -1]";
    return true;
  }

  int numLine=countLine();
  if(numLine==0) {
    FILE *fp = fopen(ori_file.c_str(), "a");
    fprintf(fp, "%lf %lf %lf %lf %lf %lf\n", js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5]);
    fclose(fp);
    return true;
  }
  else if(numLine<req.index) {
    res.error.data = "ERR: No line to delete";
    return true;
  }

  FILE *fp1 = fopen(ori_file.c_str(), "r");
  FILE *fp2 = fopen(tmp_file.c_str(), "w");
  if( !fp1 ) {
    ROS_ERROR("Cannot open file result file : %s", ori_file.c_str());
    res.error.data = "ERR: Cannot open file";
    return true;
  }
  int i=0, c=0;
  while ((c=fgetc(fp1)) != EOF) {
    if( i==req.index && req.operation == 1 ) {
      fprintf(fp2, "%lf %lf %lf %lf %lf %lf\n", js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5]);
      i++;
    }
    if(c == '\n')
      i++;
    if( req.operation == -1 )
      if( i==req.index || (i-1==req.index && c=='\n' && req.index==0))
        continue;
    fputc(c, fp2);
  }
  if( req.index == -1 && req.operation == 1)
    fprintf(fp2, "%lf %lf %lf %lf %lf %lf\n", js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5]);
  if(fp1)
    fclose(fp1);
  if(fp2)
    fclose(fp2);
  std::remove(ori_file.c_str());
  std::rename(tmp_file.c_str(), ori_file.c_str());
  return true;
}
int countLine(void) {
  FILE *fp = fopen(ori_file.c_str(), "r");
  if( !fp ) {
    ROS_ERROR("Cannot create file result file : %s", ori_file.c_str());
    FILE *fp = fopen(ori_file.c_str(), "w");
    return 0;
  }
  char c;
  int line=0;
  std::string tmp="";
  do {
    c=fgetc(fp);
    tmp += c;
    if(c == '\n') {
      line++;
    }
  } while(c!=EOF);
  fclose(fp);
  return line;
}

// cobot_msgs::ReadJointStateFile
bool readJointStateFileFunc(cobot_msgs::ReadJointStateFile::Request  &req, cobot_msgs::ReadJointStateFile::Response &res) {
  ROS_INFO("readJointStateFileFunc");
  res.error.data = "OK";
  FILE *fp = fopen(ori_file.c_str(), "r");
  if( !fp ) {
    ROS_ERROR("Cannot create file result file : %s", ori_file.c_str());
    res.error.data = "ERROR: Cannot open file";
    return true;
  }
  std::vector<std::string> v_str;
  char tmp[128];
  int i=0;
  while (1) {
    if (fgets(tmp, 128, fp) == NULL)
      break;
    i++;
    std::string s = tmp;
    boost::algorithm::split( v_str, s, boost::algorithm::is_any_of( " ,\n" ), boost::token_compress_on );
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name = {"J1","J2","J3","J4","J5","J6"};

    if(v_str.size()>=6) {
      js.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      for(int k=0; k<6; k++) {
        std::string::size_type sz;     // alias of size_t
        js.position[k] = std::stof(v_str[k], &sz);
      }
    }
    ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5]);
    res.js.push_back(js);
  }
  return true;
}

bool changeModeFunc(cobot_msgs::ChangeMode::Request  &req, cobot_msgs::ChangeMode::Response &res) {
  cobot_dynamixel_driver::change_key srv;
  srv.request.key = gen_random(8);
  ROS_INFO("srv.request.key = %s", srv.request.key.c_str());
  ros::ServiceClient client_change_key = rn->n->serviceClient<cobot_dynamixel_driver::change_key>("/cobot/cobot_dynamixel_driver/change_key");
  if (client_change_key.call(srv)) ;
  ROS_INFO("srv.response.status = %d", srv.response.status);
  if (srv.response.status) {
    DRIVER_KEY = srv.request.key;
    COBOT_MODE = req.mode;
    res.key = srv.request.key;
    res.error = "OK";
  }
  else {
    res.error = "ERROR";
  }
  return true;
}

std::string gen_random(const int len) {
  static const char alphanum[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";

  char s[len+1];
  for (int i = 0; i < len; ++i)
    s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
  s[len] = 0;
  std::string str(s);
  return str;
}

void callback_pick(const geometry_msgs::PoseArray &p) {
  ROS_INFO("callback_pick : state = %d", state_pp);
  if(state_pp == 0) {
    pp = p;
    state_pp = 1;
  }
  ROS_INFO("callback_pick end : state = %d", state_pp);
}

geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose, std::string &error) {
	moveit_msgs::GetPositionFK msg;
	msg.request.header.stamp = ros::Time::now();
	msg.request.fk_link_names = {"tool0"};
	msg.request.robot_state.joint_state = _js;
	error = "OK";
  ros::ServiceClient srv_fk = rn->n->serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
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
		error = "Failed to call service compute_fk";
	}
	return msg.response.pose_stamped[0].pose;
}

void getIK(geometry_msgs::Pose &_ps) {
	moveit_msgs::GetPositionIK msg;
  msg.request.ik_request.group_name = "arm";
  // msg.request.ik_request.robot_state.joint_state = js;
  msg.request.ik_request.pose_stamped.pose = _ps;
  // msg.request.ik_request.timeout = 1;
  msg.request.ik_request.attempts = 0;

  ros::ServiceClient srv_ik = rn->n->serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	if(srv_ik.call(msg))
	{
		; //msg.response.solution.joint_state;
	}
	else {
		ROS_ERROR("Failed to call service compute_fk");
	}

  ROS_INFO("JS : %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", msg.response.solution.joint_state.position[0]
                                                    , msg.response.solution.joint_state.position[1]
                                                    , msg.response.solution.joint_state.position[2]
                                                    , msg.response.solution.joint_state.position[3]
                                                    , msg.response.solution.joint_state.position[4]
                                                    , msg.response.solution.joint_state.position[5]);
}

void callback_pp_done(const std_msgs::Bool &p) {
  if(state_pp == 2) {
    std_msgs::Bool send;
    send.data = true;
    pub_vacuum.publish(send);
    state_pp = 3;
  }
  else if(state_pp == 4) {
    std_msgs::Bool send;
    send.data = false;
    pub_vacuum.publish(send);
    state_pp = 5;
  }
  else if(state_pp == 6)
    state_pp = 0;
}
