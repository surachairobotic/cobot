
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

  if( group_name!="arm" ){
    ROS_ERROR("group name must be 'arm' : %s", group_name.c_str());
    return false;
  }
  if( base_name!="base_link" ){
    ROS_ERROR("base name must be 'base_link' : %s", base_name.c_str());
    return false;
  }
  if( tip_name!="L5" ){
    ROS_ERROR("tip name must be 'L5' : %s", tip_name.c_str());
    return false;
  }

  while(!loadRobotModel(private_handle,robot_model,xml_string) && private_handle.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
  }

  std::vector<urdf::JointSharedPtr> joints;

  urdf::LinkConstSharedPtr link = robot_model.getRoot();
  if(link){
    link_names.push_back(link->name);
  }
  while(!link->child_joints.empty()){
//    ROS_INFO("link : %s", link->name.c_str());
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
//    ROS_INFO("joint : %s , %d", joint->name.c_str(), (int)joint->type);
    link_lens.push_back(joint->parent_to_joint_origin_transform.position.z);
    joints.push_back(joint);
    link_names.push_back(link->name);
    joint_names.push_back(joint->name);
    joint_axis.push_back(joint->axis);
  }

  // check link & joint name
  const std::string JOINT_NAME[] = {
      "J1",
      "J2",
      "J3",
      "J4",
      "J5",
      "J_tool0"
  }
  , LINK_NAME[] = {
      "base_link",
      "L1",
      "L2",
      "L3",
      "L4",
      "L5",
      "tool0"
  };

  if( joints.size()!=sizeof(JOINT_NAME) / sizeof(std::string)){
    ROS_ERROR("Invalid joint num : %d", (int)joints.size());
    return false;
  }
  for(int i=0;i<joints.size();i++){
    if( joints[i]->name!=JOINT_NAME[i] ){
      ROS_ERROR("joint name does not match : %s / %s", joints[i]->name.c_str()
          , JOINT_NAME[i].c_str());
      return false;
    }
  }
  if( link_names.size()!=sizeof(LINK_NAME) / sizeof(std::string)){
    ROS_ERROR("Invalid link num : %d", (int)link_names.size());
    return false;
  }
  for(int i=0;i<link_names.size();i++){
    if( link_names[i]!=LINK_NAME[i] ){
      ROS_ERROR("link name does not match : %s / %s", link_names[i].c_str()
          , LINK_NAME[i].c_str());
      return false;
    }
  }
  for(int i=0;i<link_names.size();i++)
    ROS_INFO("link : %s", link_names[i].c_str());
  link_names.pop_back();
  for(int i=0;i<joint_names.size();i++)
    ROS_INFO("joint : %s", joint_names[i].c_str());
  joint_names.pop_back();

  {
    // each joint's tf
    joint_tfs.clear();
    for(int i=0;i<5;i++){
      const urdf::Rotation &q = joints[i]->parent_to_joint_origin_transform.rotation;
      const urdf::Vector3 &p = joints[i]->parent_to_joint_origin_transform.position;
      joint_tfs.push_back(tf::Transform(
          tf::Quaternion(q.x,q.y,q.z,q.w),
          tf::Vector3(p.x,p.y,p.z)));
    }

    // create tf from base link to origin
    const urdf::Pose &p = joints[0]->parent_to_joint_origin_transform;
    tf_origin_inv = tf::Transform(tf::Quaternion(0,0,0,1)
        , tf::Vector3(p.position.x,p.position.y,p.position.z)).inverse();
/*    double roll, pitch, yaw;
    tf::Quaternion q;
    p.rotation.getRPY(roll, pitch, yaw);
    q.setRPY(roll,0,0);
    tf_origin_inv = tf::Transform(q
            , tf::Vector3(p.position.x,p.position.y,p.position.z)).inverse();*/
/*    tf_origin_inv = tf::Transform( tf::Quaternion(p.rotation.x,p.rotation.y
        ,p.rotation.z,p.rotation.w)
        , tf::Vector3(p.position.x,p.position.y,p.position.z)).inverse();*/
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
  // move J1 to origin
  tf::Transform t1(tf::Quaternion(ik_pose.orientation.x,ik_pose.orientation.y
      ,ik_pose.orientation.z,ik_pose.orientation.w)
      , tf::Vector3(ik_pose.position.x,ik_pose.position.y,ik_pose.position.z))
      , t2;
  t2 = tf_origin_inv * t1;
  tf::Vector3 p = t2.getOrigin();
  tf::Quaternion qt = t2.getRotation();
  const double x = -p.y(), y = p.x(), z = p.z();
  const double L2 = link_lens[2], L3 = link_lens[3], Lz = link_lens[4];

  // calculate joint angles
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(qt);
//  mat = mat.transpose();
  mat.getEulerYPR(yaw, pitch, roll);
/*
  ROS_INFO("tip : %lf, %lf, %lf"
      , t1.getOrigin().x(), t1.getOrigin().y(), t1.getOrigin().z());
  ROS_INFO("origin : %lf, %lf, %lf"
      , tf_origin_inv.getOrigin().x(), tf_origin_inv.getOrigin().y(), tf_origin_inv.getOrigin().z());
  ROS_INFO("xyz : %lf , %lf , %lf", x,y,z);
  ROS_INFO("xyzw : %lf , %lf , %lf , %lf",qt.x(), qt.y(), qt.z(), qt.w());
  ROS_INFO("YPR : %lf , %lf , %lf",yaw, pitch,roll);
*/
  double q[6] = {0};
  double ang;
  q[0] = atan2(y,x);
  {
    double x0 = joint_tfs[0].getOrigin().x(), y0 = joint_tfs[0].getOrigin().y();
//    double x0 = joint_tfs[0].position.x, y0 = joint_tfs[0].position.y;
    tf::Vector3 x1 = mat.getColumn(2)
        , x2(t2.getOrigin().x(), t2.getOrigin().y(), 0)
        , x3 = x1.cross(x2), x4, n(0,0,-1);

    // check if ee direction is valid
    {
      tf::Vector3 xx(x1.x(), x1.y(), 0);
      double len = xx.length();
      if( len>0.1 ){
        xx/= len;
        if( fabs(xx.dot(x2.normalized())) < 0.9 ){
          ROS_ERROR("Invalid ee direction : \n tip : %lf %lf %lf\n link : %lf %lf %lf"
              , x1.x(), x1.y(), x1.z(), x2.x(), x2.y(), x2.z());
          return false;
        }
      }
    }
    // when ee direction is parallel to the ground
    if( fabs(x3.length())<0.00001 ){
      x4.setValue(0.0,0.0,-1.0);
    }
    else{
      x3.normalize();
      x4 = x3.cross(x1);
    }

    {
      double d = x1.dot(n);
      if( d<-1.0 )
        d = -1.0;
      else if( d>1.0 )
        d = 1.0;
      ang = acos(d);
    }

    double c = x4.dot(mat.getColumn(0))
        , s = x4.dot(mat.getColumn(1));
    if(x1[2]>0){
      c = -c;
      s = -s;
    }
    q[4] = -atan2(c,s);
/*    ROS_INFO("cs : %lf , %lf, R22 : %lf", c, s, x1[2]);
    ROS_INFO("x1 : %lf , %lf, %lf", x1.x(), x1.y(), x1.z());
    ROS_INFO("x4 : %lf , %lf, %lf", x4.x(), x4.y(), x4.z());*/

    if((fabs(x1[0])>0.0001 && fabs(x2[0])>0.0001 && x1[0]*x2[0]<0) ||
       (fabs(x1[1])>0.0001 && fabs(x2[1])>0.0001 && x1[1]*x2[1]<0) )
      ang = fabs(ang);
    else
      ang = -fabs(ang);

  }
//  q[4] = yaw - q[0];
  {
  //  double ang = M_PI - roll; // tip's angle
    double x2 = sqrt(x*x + y*y) - link_lens[4] * sin(-ang)
        , z2 = z + (link_lens[4] * cos(-ang));
    double A = link_lens[2], B = link_lens[3], C = sqrt(x2*x2 + z2*z2);
    double cos_b = (B*B + C*C -A*A)/(2*B*C)
        , cos_a = (C-B*cos_b)/A
        , a = acos(cos_a)
        , b = acos(cos_b);
//    if( C - A*cos_a < 0.0 )
//      b = M_PI - b;
    if( std::isnan(a) ){
      ROS_ERROR("a nan");
      ROS_INFO("xyz : %lf, %lf, %lf", x, y, z);
      ROS_INFO("ang : %lf", ang);
      ROS_INFO("xz2 : %lf, %lf", x2, z2);
      ROS_INFO("ABC : %lf, %lf, %lf", A, B, C);
      return false;
    }
    if( fabs(cos_a)>1 || fabs(cos_b)>1 ){
      ROS_ERROR("Invalid A,B angle : cos_a = %.3lf, cos_b = %.3lf", cos_a, cos_b);
      return false;
    }
/*    ROS_INFO("err : %lf , %lf", A*sin(a) - B*sin(b), A*cos(a) + B*cos(b) - C);
    ROS_INFO("cos: %lf , %lf , %lf , %lf", cos_a, a, cos_b, b);
    ROS_INFO("cos err: %lf , %lf", cos_a - cos(a), cos_b - cos(b));*/

    q[1] = M_PI*0.5 - (atan2(z2,x2) + a);
    q[2] = M_PI*0.5 - (M_PI - a - b);
    // M_PI*0.5 - q[1] + M_PI*0.5 - q[2] + M_PI*0.5 - q[3] = ang;
    q[3] = (M_PI*2 - q[1] - q[2]) + ang;
    
/*
    ROS_INFO("ang : %lf", ang);
    ROS_INFO("ab : %lf, %lf", a,b);
    ROS_INFO("xz2 : %lf, %lf", x2, z2);
    ROS_INFO("ABC : %lf, %lf, %lf", A, B, C);*/

  }

/*  for(int i=0;i<link_lens.size();i++){
    ROS_INFO("L %d : %lf", i, link_lens[i]);
  }*/

  solution.resize(5);
  for(int i=0;i<5;i++){
    while(q[i] > M_PI) q[i] -= 2*M_PI;
    while(q[i] < -M_PI) q[i] += 2*M_PI;
    if( i==3 ){
      if( q[i] > M_PI*0.5 ){
        q[i]-= 2*M_PI;
      }
    }
    solution[i] = q[i];
  }
  for(int i=0;i<5;i++){
    ROS_INFO("q %d : %lf", i, q[i]);
  }
  return true;
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool AffbotKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
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
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool AffbotKinematics::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
{
  double q[5] = {0};
//  std::vector<double> q(joint_tfs.size());
//  for(int i=joint_tfs.size()-1;i>=0;i--)
//    q[i] = 0;
  if( link_names.size()!=dimension_){
    ROS_ERROR("Invalid link num  : %d"
        , (int)link_names.size());
    return false;
  }
  if( link_names.size()!=joint_angles.size()){
    ROS_ERROR("Link and joint size does not match : %d / %d"
        , (int)link_names.size(), (int)joint_angles.size());
    return false;
  }
  for(int i=0;i<link_names.size();i++){
    int j = 0;
    for(;j<this->link_names.size();j++){
      if( link_names[i]==this->link_names[j]){
        break;
      }
    }
    if( j>=this->link_names.size() ){
      ROS_ERROR("Link name does not match : %s", link_names[i].c_str());
      return false;
    }
    q[j] = joint_angles[i];
  }
//  for(int i=0;i<dimension_;i++)
//      ROS_INFO("q[%d] :%lf", i, q[i]);

  tf::Transform t;
  t.setIdentity();
  for(int i=0;i<joint_tfs.size();i++){
    const urdf::Vector3 &axis = joint_axis[i];
    tf::Quaternion qq(tf::Vector3(axis.x,axis.y,axis.z), q[i]);
    t = t * joint_tfs[i] * tf::Transform( qq, tf::Vector3(0,0,0));
//    ROS_INFO("t[%d] : %lf , %lf , %lf", i, t.getOrigin().x()
//        , t.getOrigin().y(), t.getOrigin().z());
  }
//  tf::Transform( q, tf::Vector3(0,0,0)) *

  {
    geometry_msgs::Pose pose;
    const tf::Vector3 &p = t.getOrigin();
    const tf::Quaternion &q = t.getRotation();
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    poses.clear();
    poses.push_back(pose);

//    ROS_INFO("pos  : %lf, %lf, %lf", p.x(), p.y(), p.z());
//    ROS_INFO("xyzw : %lf, %lf, %lf, %lf", q.x(), q.y(), q.z(), q.w());
  }
  return true;
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
