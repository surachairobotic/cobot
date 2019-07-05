#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "cobot_msgs/EditJointStateFile.h"
#include "cobot_msgs/ReadJointStateFile.h"
#include "cobot_msgs/ChangeMode.h"
#include "cobot_dynamixel_driver/change_key.h"

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>

std::string COBOT_MODE = "TEACH_MODE";
std::string DRIVER_KEY = "";
sensor_msgs::JointState js;
std::string ori_file = "/home/mtec/catkin_ws/src/cobot/cobot_core/data/joint_state_file.txt";
std::string tmp_file = "/home/mtec/catkin_ws/src/cobot/cobot_core/data/temp_file.txt";
ros::ServiceClient client_change_key;

void callback_js(const sensor_msgs::JointState &js);
bool editJointStateFileFunc(cobot_msgs::EditJointStateFile::Request  &req, cobot_msgs::EditJointStateFile::Response &res);
bool readJointStateFileFunc(cobot_msgs::ReadJointStateFile::Request  &req, cobot_msgs::ReadJointStateFile::Response &res);
bool changeModeFunc(cobot_msgs::ChangeMode::Request  &req, cobot_msgs::ChangeMode::Response &res);
int countLine(void);
std::string gen_random(const int len);


int main(int argc, char **argv) {
  ros::init(argc, argv, "cobot_core");
  ros::NodeHandle n;
  ros::Subscriber sub_js = n.subscribe("/cobot/joint_states", 100, callback_js);

  ros::ServiceServer service_edit_js_file = n.advertiseService("/cobot/cobot_core/edit_js_file", editJointStateFileFunc);
  ros::ServiceServer service_read_js_file = n.advertiseService("/cobot/cobot_core/read_js_file", readJointStateFileFunc);
  ros::ServiceServer service_change_mode  = n.advertiseService("/cobot/cobot_core/change_mode" , changeModeFunc);

  client_change_key = n.serviceClient<cobot_dynamixel_driver::change_key>("/cobot/cobot_dynamixel_driver/change_key");

  ROS_INFO("COBOT_CORE : START");
  ros::Rate loop_rate(100);
  while(ros::ok()) {
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
  if(COBOT_MODE.compare("TEACH_MODE") != 0) {
    res.error.data = "ERR: CobotMode is not TEACH_MODE";
    ROS_INFO("res.error.data");
    return true;
  }
  else if( !(req.operation == 1 || req.operation == -1) ) {
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
  if (client_change_key.call(srv)) ;
  if (srv.response.status) {
    DRIVER_KEY = srv.request.key;
    COBOT_MODE = req.mode;
    res.key = srv.request.key;
    res.error = "OK";
  }
  else {
    res.error = "ERROR";
  }
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
