#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/exceptions/exceptions.h>
#include <moveit/robot_state/attached_body.h>
#include <octomap_msgs/conversions.h>
#include <memory>
#include <set>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit_resources/config.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <boost/filesystem.hpp>


// Need a floating point tolerance when checking joint limits, in case the joint starts at limit
const double LIMIT_TOLERANCE = 0.01;
/// \brief Search modes for searchPositionIK(), see there
enum SEARCH_MODE
{
  OPTIMIZE_FREE_JOINT = 1,
  OPTIMIZE_MAX_JOINT = 2
};

namespace cobot_kinematics_plugin
{
#define IKFAST_NO_MAIN  // Don't include main() from IKFast

/// \brief The types of inverse kinematics parameterizations supported.
///
/// The minimum degree of freedoms required is set in the upper 4 bits of each type.
/// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
/// The lower bits contain a unique id of the type.
enum IkParameterizationType
{
  IKP_None = 0,
  IKP_Transform6D = 0x67000001,    ///< end effector reaches desired 6D transformation
  IKP_Rotation3D = 0x34000002,     ///< end effector reaches desired 3D rotation
  IKP_Translation3D = 0x33000003,  ///< end effector origin reaches desired 3D translation
  IKP_Direction3D = 0x23000004,    ///< direction on end effector coordinate system reaches desired direction
  IKP_Ray4D = 0x46000005,          ///< ray on end effector coordinate system reaches desired global ray
  IKP_Lookat3D = 0x23000006,       ///< direction on end effector coordinate system points to desired 3D position
  IKP_TranslationDirection5D = 0x56000007,  ///< end effector origin and direction reaches desired 3D translation and
  /// direction. Can be thought of as Ray IK where the origin of the ray must
  /// coincide.
  IKP_TranslationXY2D = 0x22000008,             ///< 2D translation along XY plane
  IKP_TranslationXYOrientation3D = 0x33000009,  ///< 2D translation along XY plane and 1D rotation around Z axis. The
  /// offset of the rotation is measured starting at +X, so at +X is it 0,
  /// at +Y it is pi/2.
  IKP_TranslationLocalGlobal6D = 0x3600000a,  ///< local point on end effector origin reaches desired 3D global point

  IKP_TranslationXAxisAngle4D = 0x4400000b,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with x-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationYAxisAngle4D = 0x4400000c,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with y-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationZAxisAngle4D = 0x4400000d,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with z-axis like a cone, angle is from
  /// 0-pi. Axes are defined in the manipulator base link's coordinate system.

  IKP_TranslationXAxisAngleZNorm4D = 0x4400000e,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to z-axis and be rotated at a
  /// certain angle starting from the x-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationYAxisAngleXNorm4D = 0x4400000f,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to x-axis and be rotated at a
  /// certain angle starting from the y-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationZAxisAngleYNorm4D = 0x44000010,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to y-axis and be rotated at a
  /// certain angle starting from the z-axis (defined in the manipulator
  /// base link's coordinate system)

  IKP_NumberOfParameterizations = 16,  ///< number of parameterizations (does not count IKP_None)

  IKP_VelocityDataBit =
      0x00008000,  ///< bit is set if the data represents the time-derivate velocity of an IkParameterization
  IKP_Transform6DVelocity = IKP_Transform6D | IKP_VelocityDataBit,
  IKP_Rotation3DVelocity = IKP_Rotation3D | IKP_VelocityDataBit,
  IKP_Translation3DVelocity = IKP_Translation3D | IKP_VelocityDataBit,
  IKP_Direction3DVelocity = IKP_Direction3D | IKP_VelocityDataBit,
  IKP_Ray4DVelocity = IKP_Ray4D | IKP_VelocityDataBit,
  IKP_Lookat3DVelocity = IKP_Lookat3D | IKP_VelocityDataBit,
  IKP_TranslationDirection5DVelocity = IKP_TranslationDirection5D | IKP_VelocityDataBit,
  IKP_TranslationXY2DVelocity = IKP_TranslationXY2D | IKP_VelocityDataBit,
  IKP_TranslationXYOrientation3DVelocity = IKP_TranslationXYOrientation3D | IKP_VelocityDataBit,
  IKP_TranslationLocalGlobal6DVelocity = IKP_TranslationLocalGlobal6D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngle4DVelocity = IKP_TranslationXAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngle4DVelocity = IKP_TranslationYAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngle4DVelocity = IKP_TranslationZAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngleZNorm4DVelocity = IKP_TranslationXAxisAngleZNorm4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngleXNorm4DVelocity = IKP_TranslationYAxisAngleXNorm4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngleYNorm4DVelocity = IKP_TranslationZAxisAngleYNorm4D | IKP_VelocityDataBit,

  IKP_UniqueIdMask = 0x0000ffff,   ///< the mask for the unique ids
  IKP_CustomDataBit = 0x00010000,  ///< bit is set if the ikparameterization contains custom data, this is only used
  /// when serializing the ik parameterizations
};

// struct for storing and sorting solutions
struct LimitObeyingSol
{
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};

// Code generated by IKFast56/61
//#include "cobot_arm_ikfast_solver.cpp"
int GetIkType() { return 0x67000001; }
int GetNumJoints() { return 6; }
int GetNumFreeParameters() { return 0; }
int* GetFreeParameters() { return NULL; }


class CobotKinematic : public kinematics::KinematicsBase
{
  std::vector<std::string> joint_names_;
  std::vector<double> joint_min_vector_;
  std::vector<double> joint_max_vector_;
  std::vector<bool> joint_has_limits_vector_;
  std::vector<std::string> link_names_;
  const size_t num_joints_;
  std::vector<int> free_params_;
  bool active_;  // Internal variable that indicates whether solvers are configured and ready
  const std::string name_{ "ikfast" };

  const std::vector<std::string>& getJointNames() const
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const
  {
    return link_names_;
  }

public:

	// std::vector<double> L{0.184, 0.27203, 0.25, 0.0, 0.161};
	std::vector<double> L{0.184, 0.27203, 0.25, 0.0, 0.111+0.081};
	//std::vector<double> L{0.184, 0.27203, 0.25, 0.109, 0.12136};
	std::vector<double> a, alpha, d, theta;
	double H_PI;

  /** @class
   *  @brief Interface for an IKFast kinematics plugin
   */
  CobotKinematic() : num_joints_(GetNumJoints()), active_(false)
  {
    srand(time(NULL));
    supported_methods_.push_back(kinematics::DiscretizationMethods::NO_DISCRETIZATION);
    supported_methods_.push_back(kinematics::DiscretizationMethods::ALL_DISCRETIZED);
    supported_methods_.push_back(kinematics::DiscretizationMethods::ALL_RANDOM_SAMPLED);
		H_PI = M_PI/2.0;
	//	std::vector<double> L{0.184, 0.27203, 0.141, 0.109, 0.120, 0.041};
		a = {0, 0, L[1], 0, 0, 0};
		alpha = {0, -H_PI, 0, -H_PI, H_PI, -H_PI};
		d = {L[0], 0, 0, L[2]+L[3], 0, L[4]};

		ROS_WARN("CobotKinematic() : constructor");
/*
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		kmodel_ = robot_model_loader.getModel();
		if( kmodel_ == NULL )
		  ROS_WARN("kmodel_ = NULL");
		else
		  ROS_WARN("kmodel_ is OK !!!");
*/
  }

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */

  // Returns the IK solution that is within joint limits closest to ik_seed_state
  bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, compute the set joint angles solutions that are able to reach it.
   *
   * This is a default implementation that returns only one solution and so its result is equivalent to calling
   * 'getPositionIK(...)' with a zero initialized seed.
   *
   * @param ik_poses  The desired pose of each tip link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solutions A vector of vectors where each entry is a valid joint solution
   * @param result A struct that reports the results of the query
   * @param options An option struct which contains the type of redundancy discretization used. This default
   *                implementation only supports the KinmaticSearches::NO_DISCRETIZATION method; requesting any
   *                other will result in failure.
   * @return True if a valid set of solutions was found, false otherwise.
   */
  bool getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                     std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result,
                     const kinematics::KinematicsQueryOptions& options) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
												const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
												const std::vector<double>& ik_seed_state, double timeout,
                        const std::vector<double>& consistency_limits,
												std::vector<double>& solution,
                        moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
												const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
   * around those specified in the seed state are admissible and need to be searched.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
												const std::vector<double>& ik_seed_state,
												double timeout,
                        const std::vector<double>& consistency_limits,
												std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
												moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   *
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const;

  /**
   * @brief Sets the discretization value for the redundant joint.
   *
   * Since this ikfast implementation allows for one redundant joint then only the first entry will be in the
   *discretization map will be used.
   * Calling this method replaces previous discretization settings.
   *
   * @param discretization a map of joint indices and discretization value pairs.
   */
  void setSearchDiscretization(const std::map<int, double>& discretization);

  /**
   * @brief Overrides the default method to prevent changing the redundant joints
   */
  bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices);

	robot_model::RobotModelPtr kmodel_;
	collision_detection::CollisionRobotPtr crobot_;

private:
  bool initialize(const std::string& robot_description, const std::string& group_name, const std::string& base_name,
                  const std::string& tip_name, double search_discretization);

  /**
   * @brief Calls the IK solver from IKFast
   * @return The number of solutions found
   */
  int solve(KDL::Frame& pose_frame, const std::vector<double>& vfree, std::vector<std::vector<double>>& solutions) const;

  /**
   * @brief Gets a specific solution from the set
   */
  void getSolution(const std::vector<std::vector<double>>& solutions, int i, std::vector<double>& solution) const;

  /**
   * @brief Gets a specific solution from the set with joints rotated 360° to be near seed state where possible
   */
  void getSolution(const std::vector<std::vector<double>>& solutions, const std::vector<double>& ik_seed_state, int i,
                   std::vector<double>& solution) const;

  double harmonize(const std::vector<double>& ik_seed_state, std::vector<double>& solution) const;
  // void getOrderedSolutions(const std::vector<double> &ik_seed_state, std::vector<std::vector<double> >& solslist);
  void getClosestSolution(const std::vector<std::vector<double>>& solutions, const std::vector<double>& ik_seed_state,
                          std::vector<double>& solution) const;
  void fillFreeParams(int count, int* array);
  bool getCount(int& count, const int& max_count, const int& min_count) const;

  /**
  * @brief samples the designated redundant joint using the chosen discretization method
  * @param  method              An enumeration flag indicating the discretization method to be used
  * @param  sampled_joint_vals  Sampled joint values for the redundant joint
  * @return True if sampling succeeded.
  */
  bool sampleRedundantJoint(kinematics::DiscretizationMethod method, std::vector<double>& sampled_joint_vals) const;
};  // end class

bool CobotKinematic::initialize(const std::string& robot_description, const std::string& group_name,
                                        const std::string& base_name, const std::string& tip_name,
                                        double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name, search_discretization);

  ros::NodeHandle node_handle("~/" + group_name);

  std::string robot;
  lookupParam("robot", robot, std::string());

  // IKFast56/61
  fillFreeParams(GetNumFreeParameters(), GetFreeParameters());

  if (free_params_.size() > 1)
  {
    ROS_FATAL("Only one free joint parameter supported!");
    return false;
  }
  else if (free_params_.size() == 1)
  {
    redundant_joint_indices_.clear();
    redundant_joint_indices_.push_back(free_params_[0]);
    KinematicsBase::setSearchDiscretization(DEFAULT_SEARCH_DISCRETIZATION);
  }

  urdf::Model robot_model;
  std::string xml_string;

  std::string urdf_xml, full_urdf_xml;
  lookupParam("urdf_xml", urdf_xml, robot_description);
  node_handle.searchParam(urdf_xml, full_urdf_xml);

  ROS_DEBUG_NAMED(name_, "Reading xml file from parameter server");
  if (!node_handle.getParam(full_urdf_xml, xml_string))
  {
    ROS_FATAL_NAMED(name_, "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  robot_model.initString(xml_string);

  ROS_DEBUG_STREAM_NAMED(name_, "Reading joints and links from URDF");

  urdf::LinkConstSharedPtr link = robot_model.getLink(getTipFrame());
  while (link->name != base_frame_ && joint_names_.size() <= num_joints_)
  {
    ROS_DEBUG_NAMED(name_, "Link %s", link->name.c_str());
    link_names_.push_back(link->name);
    urdf::JointSharedPtr joint = link->parent_joint;
    if (joint)
    {
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Adding joint " << joint->name);

        joint_names_.push_back(joint->name);
        float lower, upper;
        int hasLimits;
        if (joint->type != urdf::Joint::CONTINUOUS)
        {
          if (joint->safety)
          {
            lower = joint->safety->soft_lower_limit;
            upper = joint->safety->soft_upper_limit;
          }
          else
          {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else
        {
          lower = -M_PI;
          upper = M_PI;
          hasLimits = 0;
        }
        if (hasLimits && 0)
        {
          joint_has_limits_vector_.push_back(true);
          joint_min_vector_.push_back(lower);
          joint_max_vector_.push_back(upper);
        }
        else
        {
          joint_has_limits_vector_.push_back(false);
          joint_min_vector_.push_back(-M_PI);
          joint_max_vector_.push_back(M_PI);
        }
      }
    }
    else
    {
      ROS_WARN_NAMED(name_, "no joint corresponding to %s", link->name.c_str());
    }
    link = link->getParent();
  }

  if (joint_names_.size() != num_joints_)
  {
    ROS_FATAL_STREAM_NAMED(name_, "Joint numbers mismatch: URDF has " << joint_names_.size() << " and IKFast has "
                                                                      << num_joints_);
    return false;
  }

  std::reverse(link_names_.begin(), link_names_.end());
  std::reverse(joint_names_.begin(), joint_names_.end());
  std::reverse(joint_min_vector_.begin(), joint_min_vector_.end());
  std::reverse(joint_max_vector_.begin(), joint_max_vector_.end());
  std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

  for (size_t i = 0; i < num_joints_; ++i)
    ROS_DEBUG_STREAM_NAMED(name_, joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " "
                                                  << joint_has_limits_vector_[i]);

  active_ = true;
  return true;
}

void CobotKinematic::setSearchDiscretization(const std::map<int, double>& discretization)
{
  if (discretization.empty())
  {
    ROS_ERROR("The 'discretization' map is empty");
    return;
  }

  if (redundant_joint_indices_.empty())
  {
    ROS_ERROR_STREAM("This group's solver doesn't support redundant joints");
    return;
  }

  if (discretization.begin()->first != redundant_joint_indices_[0])
  {
    std::string redundant_joint = joint_names_[free_params_[0]];
    ROS_ERROR_STREAM("Attempted to discretize a non-redundant joint "
                     << discretization.begin()->first << ", only joint '" << redundant_joint << "' with index "
                     << redundant_joint_indices_[0] << " is redundant.");
    return;
  }

  if (discretization.begin()->second <= 0.0)
  {
    ROS_ERROR_STREAM("Discretization can not takes values that are <= 0");
    return;
  }

  redundant_joint_discretization_.clear();
  redundant_joint_discretization_[redundant_joint_indices_[0]] = discretization.begin()->second;
}

bool CobotKinematic::setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices)
{
  ROS_ERROR_STREAM("Changing the redundant joints isn't permitted by this group's solver ");
  return false;
}

int CobotKinematic::solve(KDL::Frame& pose_frame, const std::vector<double>& vfree,
                                  std::vector<std::vector<double>>& solutions) const
{
	ROS_INFO("CobotKinematic::solve");
  solutions.clear();
	geometry_msgs::Pose pose;
	ROS_INFO("CobotKinematic::solve-A");
	tf::PoseKDLToMsg(pose_frame, pose);
	ROS_INFO("CobotKinematic::solve-B");

	std::vector<double> solution;
	double Px06 = pose.position.x, Py06 = pose.position.y, Pz06 = pose.position.z;

	Eigen::Quaterniond q;
	ROS_INFO("CobotKinematic::solve-C");
	double norm = sqrt(pow(pose.orientation.x, 2) + pow(pose.orientation.y, 2) + pow(pose.orientation.z, 2) + pow(pose.orientation.w, 2));
  ROS_INFO("%lf,%lf,%lf,%lf|%lf", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, norm);
	tf::quaternionMsgToEigen(pose.orientation, q);
	ROS_INFO("CobotKinematic::solve-D");
	Eigen::Matrix3d r06 = q.normalized().toRotationMatrix();

	double Px05 = Px06-(L[4]*r06(0,2)), Py05 = Py06-(L[4]*r06(1,2)), Pz05 = Pz06-(L[4]*r06(2,2));
	double Z06x = r06(0,2), Z06y = r06(1,2), Z06z = r06(2,2);
	double Y06x = r06(0,1), Y06y = r06(1,1), Y06z = r06(2,1);

	double A = Py05, B = -Px05, D = 0;
	double h1_s1 = atan2(Py05, Px05), h1_s2;
	if(h1_s1 > 0)
		  h1_s2 = h1_s1-M_PI;
	else if(h1_s1 <= 0)
		  h1_s2 = h1_s1+M_PI;
	std::vector<double> h1, h3, h5;
	h1.push_back(h1_s1);
	h1.push_back(h1_s2);

	double c1, s1, P, Q, y, h3_s1, h3_s2, PPQQ, c2, s2, h2, c23, s23, s5, c5p, c5m, c4, s4, h4, c6, s6, h6, RHSx, RHSy;
  for(int i1=0; i1<h1.size(); i1++) {
		c1 = cos(h1[i1]);
		s1 = sin(h1[i1]);
		P = Pz05-L[0];
		Q = -((Px05*c1) + (Py05*s1));
		y = ( (P*P)+(Q*Q)-(pow(L[1],2))-(pow(L[2],2)) ) / (-2*L[1]*L[2]);
		if(y <= 1.0) {
		  h3_s1 = asin(y);
		  if(y >= 0)	h3_s2 = M_PI-h3_s1;
		  else				h3_s2 = -M_PI-h3_s1;
			h3.clear();
		  h3.push_back(h3_s1);
			h3.push_back(h3_s2);

      for(int i3=0; i3<h3.size(); i3++) {
		    RHSx = L[1]-L[2]*sin(h3[i3]);
		    RHSy = L[2]*cos(h3[i3]);
		    PPQQ = (P*P)+(Q*Q);
		    c2 = ((P*RHSx) - (Q*RHSy)) / PPQQ;
		    s2 = ((-P*RHSy) - (Q*RHSx)) / PPQQ;
		    h2 = atan2(s2, c2);

		    c23 = (cos(h2)*cos(h3[i3]))-(sin(h2)*sin(h3[i3]));
		    s23 = (cos(h2)*sin(h3[i3]))+(cos(h3[i3])*sin(h2));

		    s5  = ( -( (Z06x*cos(h1[i1])*(c23)) - (Z06z*(s23)) + (Z06y*sin(h1[i1])*(c23)) ) );
		    c5p =  sqrt(1-(pow(s5,2)));
		    c5m = -sqrt(1-(pow(s5,2)));
				h5.clear();
		    h5.push_back(atan2(s5, c5p));
			  h5.push_back(atan2(s5, c5m));

		    for(int i5=0; i5<h5.size(); i5++) {
					c4 = ( (Z06z*c23)+(Z06x*cos(h1[i1])*s23)+(Z06y*sin(h1[i1])*s23) ) / ( -cos(h5[i5]) );
				  s4 = ((Z06y*cos(h1[i1]))-(Z06x*sin(h1[i1]))) / cos(h5[i5]);
				  h4 = atan2(s4, c4);

				  c6 = -(Y06x*((cos(h1[i1])*sin(h4)*s23)-(cos(h4)*sin(h1[i1])))) -(Y06y*((sin(h1[i1])*sin(h4)*s23)+(cos(h1[i1])*c4))) -(Y06z*sin(h4)*c23);
				  s6 = ( (Y06x*cos(h1[i1])*c23)  -(Y06z*s23) +(Y06y*sin(h1[i1])*c23) ) / (-cos(h5[i5]));
				  h6 = atan2(s6, c6);

					solution.clear();
				  solution.push_back(h1[i1]);
					solution.push_back(h2);
					solution.push_back(h3[i3]);
					solution.push_back(h4);
					solution.push_back(h5[i5]);
					solution.push_back(h6);
					solutions.push_back(solution);
				}
			}
		}
	}

  //ComputeIk(trans, vals, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
  std::vector<std::string> lll;
  std::vector<geometry_msgs::Pose> ppp;
  ROS_ERROR("solution.size() = %d", solutions.size());
  for(int i=0; i<solutions.size(); i++) {
    // ROS_ERROR("solve : %d", i);
    getPositionFK(lll, solutions[i], ppp);
    // printf("joint : ");
    // for(int j=0; j<solutions[i].size(); j++)
    //   printf("%lf, ", solutions[i][j]);
    // ROS_WARN("\nXYZ : %lf, %lf, %lf | %lf, %lf, %lf, %lf | %d", ppp[0].position.x, ppp[0].position.y, ppp[0].position.z, ppp[0].orientation.x, ppp[0].orientation.y, ppp[0].orientation.z, ppp[0].orientation.w, ppp.size());
  }
  return solutions.size();
}

void CobotKinematic::getSolution(const std::vector<std::vector<double>>& solutions, int i,
                                         std::vector<double>& solution) const
{
	ROS_WARN("CobotKinematic::getSolution don't used");
/*  solution.clear();
  solution.resize(num_joints_);

  // IKFast56/61
  const std::vector<double>& sol = solutions[i];
  std::vector<double> vsolfree;
*/
//  sol.GetSolution(&solution[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

  // std::cout << "solution " << i << ":" ;
  // for(int j=0;j<num_joints_; ++j)
  //   std::cout << " " << solution[j];
  // std::cout << std::endl;

  // ROS_ERROR("%f %d",solution[2],vsolfree.size());
}

void CobotKinematic::getSolution(const std::vector<std::vector<double>>& solutions,
                                         const std::vector<double>& ik_seed_state, int i,
                                         std::vector<double>& solution) const
{
	// ROS_WARN("CobotKinematic::getSolution : work");
  solution.clear();
  solution.resize(num_joints_);

  // IKFast56/61
  solution = solutions[i];

  // rotate joints by +/-360° where it is possible and useful
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    if (joint_has_limits_vector_[i])
    {
      double signed_distance = solution[i] - ik_seed_state[i];
      while (signed_distance > M_PI && solution[i] - 2 * M_PI > (joint_min_vector_[i] - LIMIT_TOLERANCE))
      {
        signed_distance -= 2 * M_PI;
        solution[i] -= 2 * M_PI;
      }
      while (signed_distance < -M_PI && solution[i] + 2 * M_PI < (joint_max_vector_[i] + LIMIT_TOLERANCE))
      {
        signed_distance += 2 * M_PI;
        solution[i] += 2 * M_PI;
      }
    }
  }
}

double CobotKinematic::harmonize(const std::vector<double>& ik_seed_state, std::vector<double>& solution) const
{
  double dist_sqr = 0;
  std::vector<double> ss = ik_seed_state;
  for (size_t i = 0; i < ik_seed_state.size(); ++i)
  {
    while (ss[i] > 2 * M_PI)
    {
      ss[i] -= 2 * M_PI;
    }
    while (ss[i] < 2 * M_PI)
    {
      ss[i] += 2 * M_PI;
    }
    while (solution[i] > 2 * M_PI)
    {
      solution[i] -= 2 * M_PI;
    }
    while (solution[i] < 2 * M_PI)
    {
      solution[i] += 2 * M_PI;
    }
    dist_sqr += fabs(ik_seed_state[i] - solution[i]);
  }
  return dist_sqr;
}

// void CobotKinematic::getOrderedSolutions(const std::vector<double> &ik_seed_state,
//                                  std::vector<std::vector<double> >& solslist)
// {
//   std::vector<double>
//   double mindist = 0;
//   int minindex = -1;
//   std::vector<double> sol;
//   for(size_t i=0;i<solslist.size();++i){
//     getSolution(i,sol);
//     double dist = harmonize(ik_seed_state, sol);
//     //std::cout << "dist[" << i << "]= " << dist << std::endl;
//     if(minindex == -1 || dist<mindist){
//       minindex = i;
//       mindist = dist;
//     }
//   }
//   if(minindex >= 0){
//     getSolution(minindex,solution);
//     harmonize(ik_seed_state, solution);
//     index = minindex;
//   }
// }

void CobotKinematic::getClosestSolution(const std::vector<std::vector<double>>& solutions,
                                                const std::vector<double>& ik_seed_state,
                                                std::vector<double>& solution) const
{
  double mindist = DBL_MAX;
  int minindex = -1;
  std::vector<double> sol;

  // IKFast56/61
  for (size_t i = 0; i < solutions.size(); ++i)
  {
    getSolution(solutions, i, sol);
    double dist = harmonize(ik_seed_state, sol);
    // std::cout << "dist[" << i << "]= " << dist << std::endl;
    if (minindex == -1 || dist < mindist)
    {
      minindex = i;
      mindist = dist;
    }
  }
  if (minindex >= 0)
  {
    getSolution(solutions, minindex, solution);
    harmonize(ik_seed_state, solution);
  }
}

void CobotKinematic::fillFreeParams(int count, int* array)
{
  free_params_.clear();
  for (int i = 0; i < count; ++i)
    free_params_.push_back(array[i]);
}

bool CobotKinematic::getCount(int& count, const int& max_count, const int& min_count) const
{
  if (count > 0)
  {
    if (-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if (count + 1 <= max_count)
    {
      count = count + 1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if (1 - count <= max_count)
    {
      count = 1 - count;
      return true;
    }
    else if (count - 1 >= min_count)
    {
      count = count - 1;
      return true;
    }
    else
      return false;
  }
}

bool CobotKinematic::getPositionFK(const std::vector<std::string>& link_names,
                                   const std::vector<double>& joint_angles,
                                   std::vector<geometry_msgs::Pose>& poses) const
{
//	ROS_INFO("CobotKinematic::getPositionFK !!!");
	std::vector<double> dh_theta;
	dh_theta = {joint_angles[0], joint_angles[1]-H_PI, joint_angles[2], joint_angles[3], joint_angles[4]+H_PI, joint_angles[5]};
/*	dh_theta.push_back(joint_angles[0]);
	dh_theta.push_back(joint_angles[1]-H_PI);
	dh_theta.push_back(joint_angles[2]);
	dh_theta.push_back(joint_angles[3]);
	dh_theta.push_back(joint_angles[4]+H_PI);
	dh_theta.push_back(joint_angles[5]);
*/
	std::vector<Eigen::Matrix4f> T;

	for(int i=0; i<dh_theta.size(); i++)
	{
		Eigen::Matrix4f x;
		x << 				cos(dh_theta[i])  				,  			-sin(dh_theta[i])				  ,         0			 	,  				a[i]				,
					sin(dh_theta[i])*cos(alpha[i])	,	cos(dh_theta[i])*cos(alpha[i])	,	-sin(alpha[i])	,	-sin(alpha[i])*d[i]	,
					sin(dh_theta[i])*sin(alpha[i])	,	cos(dh_theta[i])*sin(alpha[i])	,	 cos(alpha[i])	,	 cos(alpha[i])*d[i]	,
												 0								,								 0								,				  0				,					 	1					;

		T.push_back(x);
	}
	geometry_msgs::Pose point;
	Eigen::Matrix4f t_all = T[0];
	point.position.x = t_all(0, 3);
	point.position.y = t_all(1, 3);
	point.position.z = t_all(2, 3);

	Eigen::Matrix3f rot;
	rot(0, 0) = t_all(0, 0);
	rot(0, 1) = t_all(0, 1);
	rot(0, 2) = t_all(0, 2);

	rot(1, 0) = t_all(1, 0);
	rot(1, 1) = t_all(1, 1);
	rot(1, 2) = t_all(1, 2);

	rot(2, 0) = t_all(2, 0);
	rot(2, 1) = t_all(2, 1);
	rot(2, 2) = t_all(2, 2);
	Eigen::Quaternionf q(rot);
	point.orientation.x = q.x();
	point.orientation.y = q.y();
	point.orientation.z = q.z();
	point.orientation.w = q.w();

	poses.push_back(point);
	for(int j=1; j<T.size(); j++) {
		t_all = t_all * T[j];
		point.position.x = t_all(0, 3);
		point.position.y = t_all(1, 3);
		point.position.z = t_all(2, 3);

		Eigen::Matrix3f rot_;
		rot_(0, 0) = t_all(0, 0);
		rot_(0, 1) = t_all(0, 1);
		rot_(0, 2) = t_all(0, 2);

		rot_(1, 0) = t_all(1, 0);
		rot_(1, 1) = t_all(1, 1);
		rot_(1, 2) = t_all(1, 2);

		rot_(2, 0) = t_all(2, 0);
		rot_(2, 1) = t_all(2, 1);
		rot_(2, 2) = t_all(2, 2);
		Eigen::Quaternionf q_(rot_);
		point.orientation.x = q_.x();
		point.orientation.y = q_.y();
		point.orientation.z = q_.z();
		point.orientation.w = q_.w();

		if(j == T.size()-1)	poses.clear();
		poses.push_back(point);

	}

/////////////////////////////////////////////////////////////////////////////////////////
/*
  if (GetIkType() != IKP_Transform6D)
  {
    // ComputeFk() is the inverse function of ComputeIk(), so the format of
    // eerot differs depending on IK type. The Transform6D IK type is the only
    // one for which a 3x3 rotation matrix is returned, which means we can only
    // compute FK for that IK type.
    ROS_ERROR_NAMED(name_, "Can only compute FK for Transform6D IK type!");
    return false;
  }

  KDL::Frame p_out;
  if (link_names.size() == 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "Link names with nothing");
    return false;
  }

  if (link_names.size() != 1 || link_names[0] != getTipFrame())
  {
    ROS_ERROR_NAMED(name_, "Can compute FK for %s only", getTipFrame().c_str());
    return false;
  }

  bool valid = true;

  double eerot[9], eetrans[3];

  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED(name_, "Unexpected number of joint angles");
    return false;
  }

  double angles[num_joints_];
  for (unsigned char i = 0; i < num_joints_; i++)
  angles[i] = joint_angles[i];


  // IKFast56/61
  ComputeFk(joint_angles, eetrans, eerot);

  for (int i = 0; i < 3; ++i)
    p_out.p.data[i] = eetrans[i];

  for (int i = 0; i < 9; ++i)
    p_out.M.data[i] = eerot[i];

  poses.resize(1);
  tf::poseKDLToMsg(p_out, poses[0]);

*/
  return true;
}

bool CobotKinematic::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                      const std::vector<double>& ik_seed_state,
																			double timeout,
                                      std::vector<double>& solution,
																			moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool CobotKinematic::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                      const std::vector<double>& ik_seed_state,
																			double timeout,
                                      const std::vector<double>& consistency_limits,
                                      std::vector<double>& solution,
																			moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool CobotKinematic::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                      const std::vector<double>& ik_seed_state,
																			double timeout,
                                      std::vector<double>& solution,
																			const IKCallbackFn& solution_callback,
                                      moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool CobotKinematic::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                      const std::vector<double>& ik_seed_state,
																			double timeout,
                                      const std::vector<double>& consistency_limits,
                                      std::vector<double>& solution,
																			const IKCallbackFn& solution_callback,
                                      moveit_msgs::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
  ROS_INFO("searchPositionIK");

  /// search_mode is currently fixed during code generation
  SEARCH_MODE search_mode = OPTIMIZE_MAX_JOINT;

  // Check if there are no redundant joints
  if (free_params_.size() == 0)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "No need to search since no free params/redundant joints");

    std::vector<geometry_msgs::Pose> ik_poses(1, ik_pose);
    std::vector<std::vector<double>> solutions;
    kinematics::KinematicsResult kinematic_result;
    // Find all IK solution within joint limits
    if (!getPositionIK(ik_poses, ik_seed_state, solutions, kinematic_result, options))
    {
      ROS_ERROR("No solution whatsoever");
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // sort solutions by their distance to the seed
    std::vector<LimitObeyingSol> solutions_obey_limits;
    for (std::size_t i = 0; i < solutions.size(); ++i)
    {
      double dist_from_seed = 0.0;
      for (std::size_t j = 0; j < ik_seed_state.size(); ++j)
      {
        dist_from_seed += fabs(ik_seed_state[j] - solutions[i][j]);
      }

      solutions_obey_limits.push_back({ solutions[i], dist_from_seed });
    }
    std::sort(solutions_obey_limits.begin(), solutions_obey_limits.end());

    // check for collisions if a callback is provided
    if (!solution_callback.empty())
    {
      for (std::size_t i = 0; i < solutions_obey_limits.size(); ++i)
      {
        solution_callback(ik_pose, solutions_obey_limits[i].value, error_code);
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          solution = solutions_obey_limits[i].value;
          ROS_INFO("Solution passes callback");
          return true;
        }
      }

      ROS_ERROR("Solution has error code xx");
      return false;
    }
    else
    {
      solution = solutions_obey_limits[0].value;
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      ROS_INFO("no collision check callback provided");
      return true;  // no collision check callback provided
    }
  }

  // -------------------------------------------------------------------------------------------------
  // Error Checking
  if (!active_)
  {
    ROS_ERROR("Kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Seed state must have size " << num_joints_ << " instead of size "
                                                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    ROS_ERROR("Seed state must have size");
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != num_joints_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Consistency limits be empty or must have size " << num_joints_ << " instead of size "
                                                                                   << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    ROS_ERROR("Consistency limits be empty or must have size");
    return false;
  }

  // -------------------------------------------------------------------------------------------------
  // Initialize

  KDL::Frame frame;
  tf::poseMsgToKDL(ik_pose, frame);

  std::vector<double> vfree(free_params_.size());

  ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
  int counter = 0;

  double initial_guess = ik_seed_state[free_params_[0]];
  vfree[0] = initial_guess;

  // -------------------------------------------------------------------------------------------------
  // Handle consitency limits if needed
  int num_positive_increments;
  int num_negative_increments;

  if (!consistency_limits.empty())
  {
    // moveit replaced consistency_limit (scalar) w/ consistency_limits (vector)
    // Assume [0]th free_params element for now.  Probably wrong.
    double max_limit = fmin(joint_max_vector_[free_params_[0]], initial_guess + consistency_limits[free_params_[0]]);
    double min_limit = fmax(joint_min_vector_[free_params_[0]], initial_guess - consistency_limits[free_params_[0]]);

    num_positive_increments = (int)((max_limit - initial_guess) / search_discretization_);
    num_negative_increments = (int)((initial_guess - min_limit) / search_discretization_);
  }
  else  // no consitency limits provided
  {
    num_positive_increments = (joint_max_vector_[free_params_[0]] - initial_guess) / search_discretization_;
    num_negative_increments = (initial_guess - joint_min_vector_[free_params_[0]]) / search_discretization_;
  }

  // -------------------------------------------------------------------------------------------------
  // Begin searching

  ROS_DEBUG_STREAM_NAMED(name_, "Free param is " << free_params_[0] << " initial guess is " << initial_guess
                                                 << ", # positive increments: " << num_positive_increments
                                                 << ", # negative increments: " << num_negative_increments);
  if ((search_mode & OPTIMIZE_MAX_JOINT) && (num_positive_increments + num_negative_increments) > 1000)
    ROS_WARN_STREAM_ONCE_NAMED(name_, "Large search space, consider increasing the search discretization");

  double best_costs = -1.0;
  std::vector<double> best_solution;
  int nattempts = 0, nvalid = 0;

  while (true)
  {
    std::vector<std::vector<double>> solutions;
    int numsol = solve(frame, vfree, solutions);

    ROS_INFO("Found %d solutions from IKFast", numsol);

    // ROS_INFO("%f",vfree[0]);

    if (numsol > 0)
    {
      for (int s = 0; s < numsol; ++s)
      {
        nattempts++;
        std::vector<double> sol;
        getSolution(solutions, ik_seed_state, s, sol);

        bool obeys_limits = true;
        for (unsigned int i = 0; i < sol.size(); i++)
        {
          if (joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i]))
          {
            ROS_INFO_STREAM_NAMED(name_,"Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
            obeys_limits = false;
            break;
          }
        }
        if (obeys_limits)
        {
          getSolution(solutions, ik_seed_state, s, solution);

          // This solution is within joint limits, now check if in collision (if callback provided)
          if (!solution_callback.empty())
          {
            solution_callback(ik_pose, solution, error_code);
          }
          else
          {
            error_code.val = error_code.SUCCESS;
          }

          if (error_code.val == error_code.SUCCESS)
          {
            nvalid++;
            if (search_mode & OPTIMIZE_MAX_JOINT)
            {
              // Costs for solution: Largest joint motion
              double costs = 0.0;
              for (unsigned int i = 0; i < solution.size(); i++)
              {
                double d = fabs(ik_seed_state[i] - solution[i]);
                if (d > costs)
                  costs = d;
              }
              if (costs < best_costs || best_costs == -1.0)
              {
                best_costs = costs;
                best_solution = solution;
              }
            }
            else
						{
					    ROS_INFO("Return first feasible solution");
              // Return first feasible solution
              return true;
						}
          }
        }
      }
    }

    if (!getCount(counter, num_positive_increments, -num_negative_increments))
    {
      // Everything searched
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      break;
    }

    vfree[0] = initial_guess + search_discretization_ * counter;
    // ROS_DEBUG_STREAM_NAMED(name_,"Attempt " << counter << " with 0th free joint having value " << vfree[0]);
  }

  ROS_DEBUG_STREAM_NAMED(name_, "Valid solutions: " << nvalid << "/" << nattempts);

  if ((search_mode & OPTIMIZE_MAX_JOINT) && best_costs != -1.0)
  {
    solution = best_solution;
    error_code.val = error_code.SUCCESS;
    ROS_INFO("(search_mode & OPTIMIZE_MAX_JOINT) && best_costs != -1.0");
    return true;
  }

  // No solution found
  error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  ROS_INFO("No solution found !!!");
  return false;
}

// Used when there are no redundant joints - aka no free params
bool CobotKinematic::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_INFO("getPositionIK");

  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  if (ik_seed_state.size() < num_joints_)
  {
    ROS_ERROR_STREAM("ik_seed_state only has " << ik_seed_state.size() << " entries, this ikfast solver requires "
                                               << num_joints_);
    return false;
  }

  // Check if seed is in bound
  for (std::size_t i = 0; i < ik_seed_state.size(); i++)
  {
    // Add tolerance to limit check
    if (joint_has_limits_vector_[i] && ((ik_seed_state[i] < (joint_min_vector_[i] - LIMIT_TOLERANCE)) ||
                                        (ik_seed_state[i] > (joint_max_vector_[i] + LIMIT_TOLERANCE))))
    {
      ROS_DEBUG_STREAM_NAMED("ikseed", "Not in limits! " << (int)i << " value " << ik_seed_state[i]
                                                         << " has limit: " << joint_has_limits_vector_[i] << "  being  "
                                                         << joint_min_vector_[i] << " to " << joint_max_vector_[i]);
      return false;
    }
  }

  std::vector<double> vfree(free_params_.size());
  for (std::size_t i = 0; i < free_params_.size(); ++i)
  {
    int p = free_params_[i];
    ROS_ERROR("%u is %f", p, ik_seed_state[p]);  // DTC
    vfree[i] = ik_seed_state[p];
  }

  KDL::Frame frame;
  tf::poseMsgToKDL(ik_pose, frame);

  std::vector<std::vector<double>> solutions;
  int numsol = solve(frame, vfree, solutions);
  ROS_DEBUG_STREAM_NAMED(name_, "Found " << numsol << " solutions from IKFast");

  std::vector<LimitObeyingSol> solutions_obey_limits;

  if (numsol)
  {
    std::vector<double> solution_obey_limits;
    for (std::size_t s = 0; s < numsol; ++s)
    {
      std::vector<double> sol;
      getSolution(solutions, ik_seed_state, s, sol);
      ROS_DEBUG_NAMED(name_, "Sol %d: %e   %e   %e   %e   %e   %e", (int)s, sol[0], sol[1], sol[2], sol[3], sol[4],
                      sol[5]);

      bool obeys_limits = true;
      for (std::size_t i = 0; i < sol.size(); i++)
      {
        // Add tolerance to limit check
        if (joint_has_limits_vector_[i] && ((sol[i] < (joint_min_vector_[i] - LIMIT_TOLERANCE)) ||
                                            (sol[i] > (joint_max_vector_[i] + LIMIT_TOLERANCE))))
        {
          // One element of solution is not within limits
          obeys_limits = false;
          ROS_DEBUG_STREAM_NAMED(name_, "Not in limits! " << (int)i << " value " << sol[i] << " has limit: "
                                                          << joint_has_limits_vector_[i] << "  being  "
                                                          << joint_min_vector_[i] << " to " << joint_max_vector_[i]);
          break;
        }
      }
      if (obeys_limits)
      {
        // All elements of this solution obey limits
        getSolution(solutions, ik_seed_state, s, solution_obey_limits);
        double dist_from_seed = 0.0;
        for (std::size_t i = 0; i < ik_seed_state.size(); ++i)
        {
          dist_from_seed += fabs(ik_seed_state[i] - solution_obey_limits[i]);
        }

        solutions_obey_limits.push_back({ solution_obey_limits, dist_from_seed });
      }
    }
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(name_, "No IK solution");
  }

  // Sort the solutions under limits and find the one that is closest to ik_seed_state
  if (!solutions_obey_limits.empty())
  {
    std::sort(solutions_obey_limits.begin(), solutions_obey_limits.end());
    solution = solutions_obey_limits[0].value;
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

bool CobotKinematic::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                           const std::vector<double>& ik_seed_state,
                                           std::vector<std::vector<double>>& solutions,
                                           kinematics::KinematicsResult& result,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_INFO("getPositionIK with multiple solutions");

  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    result.kinematic_error = kinematics::KinematicErrors::SOLVER_NOT_ACTIVE;
    return false;
  }

  if (ik_poses.empty())
  {
    ROS_ERROR("ik_poses is empty");
    result.kinematic_error = kinematics::KinematicErrors::EMPTY_TIP_POSES;
    return false;
  }

  if (ik_poses.size() > 1)
  {
    ROS_ERROR("ik_poses contains multiple entries, only one is allowed");
    result.kinematic_error = kinematics::KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
  }

  if (ik_seed_state.size() < num_joints_)
  {
    ROS_ERROR_STREAM("ik_seed_state only has " << ik_seed_state.size() << " entries, this ikfast solver requires "
                                               << num_joints_);
    return false;
  }

  KDL::Frame frame;
  tf::poseMsgToKDL(ik_poses[0], frame);

  // solving ik
  std::vector<std::vector<std::vector<double>>> solution_set;
  std::vector<std::vector<double>> ik_solutions;
  std::vector<double> vfree;
  int numsol = 0;
  std::vector<double> sampled_joint_vals;
  if (!redundant_joint_indices_.empty())
  {
		ROS_INFO("if A (!redundant_joint_indices_.empty()) : in");
    // initializing from seed
    sampled_joint_vals.push_back(ik_seed_state[redundant_joint_indices_[0]]);

    // checking joint limits when using no discretization
    if (options.discretization_method == kinematics::DiscretizationMethods::NO_DISCRETIZATION &&
        joint_has_limits_vector_[redundant_joint_indices_.front()])
    {
			ROS_INFO("if AA joint_has_limits_vector_[redundant_joint_indices_.front()] : in");
      double joint_min = joint_min_vector_[redundant_joint_indices_.front()];
      double joint_max = joint_max_vector_[redundant_joint_indices_.front()];

      double jv = sampled_joint_vals[0];
      if (!((jv > (joint_min - LIMIT_TOLERANCE)) && (jv < (joint_max + LIMIT_TOLERANCE))))
      {
				ROS_INFO("if AAA jv > (joint_min - LIMIT_TOLERANCE)");
        result.kinematic_error = kinematics::KinematicErrors::IK_SEED_OUTSIDE_LIMITS;
        ROS_ERROR("ik seed is out of bounds");
        return false;
      }
    }

    // computing all solutions sets for each sampled value of the redundant joint
    if (!sampleRedundantJoint(options.discretization_method, sampled_joint_vals))
    {
			ROS_INFO("if AB !sampleRedundantJoint(options.discretization_method");
      result.kinematic_error = kinematics::KinematicErrors::UNSUPORTED_DISCRETIZATION_REQUESTED;
      return false;
    }

    for (unsigned int i = 0; i < sampled_joint_vals.size(); i++)
    {
			ROS_INFO("AC for loop [%d]", i);
      vfree.clear();
      vfree.push_back(sampled_joint_vals[i]);
      numsol += solve(frame, vfree, ik_solutions);
      solution_set.push_back(ik_solutions);
			ROS_INFO("AC numsol [%d]", numsol);
    }
  }
  else
  {
    // computing for single solution set
    numsol = solve(frame, vfree, ik_solutions);
    solution_set.push_back(ik_solutions);
		ROS_INFO("else A numsol [%d]", numsol);
  }

	ROS_INFO("solution_set.size() : %d", (int)solution_set.size());
/*
	for(int k=0; k<solution_set.size(); k++)
		for(int j=0; j<solution_set[k].size(); j++) {
			ROS_INFO("solution_set[%d][%d]", k, j);
			for(int i=0; i<solution_set[k][j].size(); i++)
				ROS_INFO("[%d][%d][%d] = %lf", k, j, i, solution_set[k][j][i]);
		}
*/
  ROS_DEBUG_STREAM_NAMED(name_, "Found " << numsol << " solutions from IKFast");
  bool solutions_found = false;
  if (numsol > 0)
  {
    /*
      Iterating through all solution sets and storing those that do not exceed joint limits.
    */
    for (unsigned int r = 0; r < solution_set.size(); r++)
    {
      ik_solutions = solution_set[r];
      numsol = ik_solutions.size();
      for (int s = 0; s < numsol; ++s)
      {
        std::vector<double> sol;
        getSolution(ik_solutions, ik_seed_state, s, sol);

        bool obeys_limits = true;
        for (unsigned int i = 0; i < sol.size(); i++)
        {
          // Add tolerance to limit check
          if (joint_has_limits_vector_[i] && ((sol[i] < (joint_min_vector_[i] - LIMIT_TOLERANCE)) ||
                                              (sol[i] > (joint_max_vector_[i] + LIMIT_TOLERANCE))))
          {
            // One element of solution is not within limits
            obeys_limits = false;
            ROS_DEBUG_STREAM_NAMED(name_, "Not in limits! " << i << " value " << sol[i] << " has limit: "
                                                            << joint_has_limits_vector_[i] << "  being  "
                                                            << joint_min_vector_[i] << " to " << joint_max_vector_[i]);
            break;
          }
        }
        if (obeys_limits)
        {
          // All elements of solution obey limits
          solutions_found = true;
          solutions.push_back(sol);
        }
      }
    }

    if (solutions_found)
    {
      result.kinematic_error = kinematics::KinematicErrors::OK;
	    ROS_INFO("solutions_found !!!");
      return true;
    }
  }
  else
  {
    ROS_ERROR("No IK solution");
  }

  result.kinematic_error = kinematics::KinematicErrors::NO_SOLUTION;
  return false;
}

bool CobotKinematic::sampleRedundantJoint(kinematics::DiscretizationMethod method,
                                                  std::vector<double>& sampled_joint_vals) const
{
  double joint_min = -M_PI;
  double joint_max = M_PI;
  int index = redundant_joint_indices_.front();
  double joint_dscrt = redundant_joint_discretization_.at(index);

  if (joint_has_limits_vector_[redundant_joint_indices_.front()])
  {
    joint_min = joint_min_vector_[index];
    joint_max = joint_max_vector_[index];
  }

  switch (method)
  {
    case kinematics::DiscretizationMethods::ALL_DISCRETIZED:
    {
      int steps = std::ceil((joint_max - joint_min) / joint_dscrt);
      for (unsigned int i = 0; i < steps; i++)
      {
        sampled_joint_vals.push_back(joint_min + joint_dscrt * i);
      }
      sampled_joint_vals.push_back(joint_max);
    }
    break;
    case kinematics::DiscretizationMethods::ALL_RANDOM_SAMPLED:
    {
      int steps = std::ceil((joint_max - joint_min) / joint_dscrt);
      steps = steps > 0 ? steps : 1;
      double diff = joint_max - joint_min;
      for (int i = 0; i < steps; i++)
      {
        sampled_joint_vals.push_back(((diff * std::rand()) / (static_cast<double>(RAND_MAX))) + joint_min);
      }
    }

    break;
    case kinematics::DiscretizationMethods::NO_DISCRETIZATION:

      break;
    default:
      ROS_ERROR_STREAM("Discretization method " << method << " is not supported");
      return false;
  }

  return true;
}

}  // end namespace

// register CobotKinematic as a KinematicsBase implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cobot_kinematics_plugin::CobotKinematic, kinematics::KinematicsBase);
