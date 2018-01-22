
#include <affbot_kinematics.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>
#include <algorithm>
#include <numeric>

#include <pluginlib/class_list_macros.h>

/*
using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;
*/

//register AffbotKinematics as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(affbot_kinematics::AffbotKinematics, kinematics::KinematicsBase);

namespace affbot_kinematics {

AffbotKinematics::AffbotKinematics():active_(false){}

bool AffbotKinematics::isActive()
{
  return active_;
}


bool loadRobotModel(ros::NodeHandle node_handle, urdf::Model &robot_model, std::string &xml_string)
{
  std::string urdf_xml,full_urdf_xml;
  node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
  node_handle.searchParam(urdf_xml,full_urdf_xml);
  TiXmlDocument xml;
  ROS_DEBUG("Reading xml file from parameter server\n");
  std::string result;
  if (node_handle.getParam(full_urdf_xml, result))
    xml.Parse(result.c_str());
  else
  {
    ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    return false;
  }
  xml_string = result;
  TiXmlElement *root_element = xml.RootElement();
  TiXmlElement *root = xml.FirstChildElement("robot");
  if (!root || !root_element)
  {
    ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
    exit(1);
  }
  robot_model.initXml(root);
  return true;

}


bool AffbotKinematics::initialize(const std::string& robot_description,
                                        const std::string& group_name,
                                        const std::string& base_name,
                                        const std::string& tip_name,
                                        double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name,search_discretization);
  urdf::Model robot_model;
  std::string xml_string;
  ros::NodeHandle private_handle("~/"+group_name);
  dimension_ = 5;

  while(!loadRobotModel(private_handle,robot_model,xml_string) && private_handle.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
  }

  std::vector<urdf::JointSharedPtr> joints;

  urdf::LinkConstSharedPtr link = robot_model.getRoot();
  while(!link->child_joints.empty()){
    ROS_INFO("link : %s", link->name.c_str());
    if(link->child_joints.size()!=1){
      ROS_ERROR("AffbotKinematicsPlugin::initialize() : link has multiple child joints : %s , %d"
          , link->name.c_str(), (int)link->child_joints.size());
      ros::Duration(0.5).sleep();
    }
    urdf::JointSharedPtr joint = link->child_joints[0];
    link = robot_model.getLink(joint->child_link_name);
    if( !link ){
      ROS_ERROR("AffbotKinematicsPlugin::initialize() : joint has no child link : %s , %s"
          , joint->name.c_str(), joint->child_link_name.c_str());
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("joint : %s , %d", joint->name.c_str(), (int)joint->type);
    link_lens.push_back(joint->parent_to_joint_origin_transform.position.z);
    joints.push_back(joint);
  }
  const std::string JOINT_NAME[] = {
      "J1",
      "J2",
      "J3",
      "J4",
      "J5",
      "J_tool0"
  };
  if( joints.size()!=sizeof(JOINT_NAME) / sizeof(std::string)){
    ROS_ERROR("Invalid joint num : %d", (int)joints.size());
    return false;
  }
  for(int i=0;i<5;i++){
    if( joints[i]->name!=JOINT_NAME[i] ){
      ROS_ERROR("joint name does not match : %s / %s", joints[i]->name.c_str()
          , JOINT_NAME[i].c_str());
    }
  }

  {
    const urdf::Pose &p = joints[0]->parent_to_joint_origin_transform;
    /*
    tf_origin = tf::Transform(tf::Quaternion(p.rotation.x,p.rotation.y
        ,p.rotation.z,p.rotation.w)
        , tf::Vector3(p.position.x,p.position.y,p.position.z));*/
    tf_origin = tf::Transform(tf::Quaternion(0,0,0,1)
            , tf::Vector3(p.position.x,p.position.y,p.position.z));
  }

  active_ = true;
  return true;
}

#define POW2(x) ((x)*(x))

bool AffbotKinematics::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  tf::Transform t1(tf::Quaternion(ik_pose.orientation.x,ik_pose.orientation.y
      ,ik_pose.orientation.z,ik_pose.orientation.w)
      , tf::Vector3(ik_pose.position.x,ik_pose.position.y,ik_pose.position.z))
      , t2;
  t2 = tf_origin.inverse() * t1;
  tf::Vector3 p = t2.getOrigin();
  tf::Quaternion qt = t2.getRotation();
  const double x = -p.y(), y = p.x(), z = p.z();
  const double L2 = link_lens[2], L3 = link_lens[3], Lz = link_lens[4];

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(qt);
  mat.getEulerYPR(yaw, pitch, roll);

  ROS_INFO("tip : %lf, %lf, %lf"
      , t1.getOrigin().x(), t1.getOrigin().y(), t1.getOrigin().z());
  ROS_INFO("origin : %lf, %lf, %lf"
      , tf_origin.getOrigin().x(), tf_origin.getOrigin().y(), tf_origin.getOrigin().z());

  if( fabs(pitch)>0.001 ){
    ROS_ERROR("Invalid tip YRP : %lf, %lf, %lf", yaw, pitch, roll);
    return false;
  }

  double q[6] = {0};

  q[0] = atan2(y,x);
  {
    double ang = M_PI - roll; // tip's angle
    double x2 = sqrt(x*x + y*y) - link_lens[4] * sin(ang)
        , z2 = z + (link_lens[4] * cos(ang));
    double A = link_lens[2], B = link_lens[3], C = sqrt(x2*x2 + z2*z2);
    double cos_b = (B*B + C*C -A*A)/(2*B*C)
        , cos_a = (C-B*cos_b)/A
        , a = acos(cos_a)
        , b = acos(cos_b);
    if( C - A*cos_a < 0.0 )
      b = M_PI - b;

    q[1] = M_PI*0.5 - (atan2(z2,x2) + a);
    q[2] = M_PI*0.5 - (M_PI - a - b);
    // M_PI*0.5 - q[1] + M_PI*0.5 - q[2] + M_PI*0.5 - q[3] = ang;
    q[3] = (M_PI*2 - q[1] - q[2]) - ang;

    ROS_INFO("ang : %lf", ang);
    ROS_INFO("ab : %lf, %lf", a,b);
    ROS_INFO("xz2 : %lf, %lf", x2, z2);
    ROS_INFO("ABC : %lf, %lf, %lf", A, B, C);
  }

  ROS_INFO("xyz : %lf , %lf , %lf", x,y,z);
  ROS_INFO("xyzw : %lf , %lf , %lf , %lf",qt.x(), qt.y(), qt.z(), qt.w());
  ROS_INFO("YPR : %lf , %lf , %lf",yaw, pitch,roll);
  for(int i=0;i<link_lens.size();i++){
    ROS_INFO("L %d : %lf", i, link_lens[i]);
  }
  for(int i=0;i<4;i++){
    ROS_INFO("q %d : %lf", i, q[i]);
  }

  solution.resize(5);
  for(int i=0;i<5;i++){
    while(q[i] > M_PI) q[i] -= 2*M_PI;
    while(q[i] < -M_PI) q[i] += 2*M_PI;
    solution[i] = q[i];
  }
  return false;
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool AffbotKinematics::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
{
  return false;
}


const std::vector<std::string>& AffbotKinematics::getJointNames() const
{
  return joint_names;
}

const std::vector<std::string>& AffbotKinematics::getLinkNames() const
{
  return link_names;
}



} // namespace
