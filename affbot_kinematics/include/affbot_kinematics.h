
#ifndef affbot_IK_NODE_H
#define affbot_IK_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
//#include <moveit/affbot_kinematics/affbot_ik_solver.h>
#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl/chainiksolver.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
//#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <geometry_msgs/PoseStamped.h>


#include <tf_conversions/tf_kdl.h>

#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
//#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/shared_ptr.hpp>

#include <moveit/kinematics_base/kinematics_base.h>

/** @brief Namespace for the AffbotKinematics*/
namespace affbot_kinematics
{
class AffbotKinematics : public kinematics::KinematicsBase
{
public:

  /**
   *  @brief Plugin-able interface to the PR2 arm kinematics
   */
  AffbotKinematics();

  /**
   *  @brief Specifies if the solver is active or not
   *  @return True if the solver is active, false otherwise.
   */
  bool isActive();

  virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                             const std::vector<double> &ik_seed_state,
                             std::vector<double> &solution,
                             moveit_msgs::MoveItErrorCodes &error_code,
                             const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::Pose> &poses) const;

  /**
   * @brief  Initialization function for the kinematics
   * @return True if initialization was successful, false otherwise
   */
  virtual bool initialize(const std::string& robot_description,
                          const std::string& group_name,
                          const std::string& base_name,
                          const std::string& tip_name,
                          double search_discretization);

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const;

protected:

  bool active_;
  urdf::Model robot_model_;
  ros::NodeHandle node_handle_, root_handle_;
  ros::ServiceServer ik_service_,fk_service_,ik_solver_info_service_,fk_solver_info_service_;
  //tf::TransformListener tf_;
  std::string root_name_;
  int dimension_;
  std::vector<std::string> link_names, joint_names;
  std::vector<double> link_lens;
  std::vector<urdf::Vector3> joint_axis;
//  std::vector<urdf::Pose> joint_tfs;
  std::vector<tf::Transform> joint_tfs;
  tf::Transform tf_origin_inv;
};
}

#endif
