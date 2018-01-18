

#include <moveit/affbot_kinematics/affbot_ik_solver.h>

using namespace Eigen;
using namespace affbot_kinematics;

AffbotIKSolver::AffbotIKSolver(const urdf::Model &robot_model,
                               const std::string &root_frame_name,
                               const std::string &tip_frame_name,
                               const double &search_discretization_angle,
                               const int &free_angle):ChainIkSolverPos()
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  root_frame_name_ = root_frame_name;
  if(!affbot_ik_.init(robot_model, root_frame_name, tip_frame_name))
    active_ = false;
  else
    active_ = true;
}

void AffbotIKSolver::getSolverInfo(moveit_msgs::KinematicSolverInfo &response)
{
  affbot_ik_.getSolverInfo(response);
}

int AffbotIKSolver::CartToJnt(const KDL::JntArray& q_init,
                              const KDL::Frame& p_in,
                              KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  std::vector<std::vector<double> > solution_ik;
  if(free_angle_ == 0)
  {
    ROS_DEBUG("Solving with free angle: %d", free_angle_);
    affbot_ik_.computeIKShoulderPan(b, q_init(0), solution_ik);
  }
  else
  {
    affbot_ik_.computeIKShoulderRoll(b, q_init(2), solution_ik);
  }

  if(solution_ik.empty())
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) solution_ik.size(); ++i)
  {
    ROS_DEBUG("Solution : %d", (int)solution_ik.size());
    for(int j=0; j < (int)solution_ik[i].size(); j++)
    {
      ROS_DEBUG("Joint %d: %f", j, solution_ik[i][j]);
    }

    double tmp_distance = computeEuclideanDistance(solution_ik[i], q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if(min_index > -1)
  {
    q_out.resize((int)solution_ik[min_index].size());
    for(int i=0; i < (int)solution_ik[min_index].size(); ++i)
    {
      q_out(i) = solution_ik[min_index][i];
    }
    return 1;
  }
  else
    return -1;
}

int AffbotIKSolver::CartToJnt(const KDL::JntArray& q_init,
                              const KDL::Frame& p_in,
                              std::vector<KDL::JntArray> &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  std::vector<std::vector<double> > solution_ik;
  KDL::JntArray q;

  if(free_angle_ == 0)
  {
    affbot_ik_.computeIKShoulderPan(b, q_init(0), solution_ik);
  }
  else
  {
    affbot_ik_.computeIKShoulderRoll(b, q_init(2), solution_ik);
  }

  if(solution_ik.empty())
    return -1;

  q.resize(7);
  q_out.clear();
  for(int i=0; i< (int) solution_ik.size(); ++i)
  {
    for(int j=0; j < 7; j++)
    {
      q(j) = solution_ik[i][j];
    }
    q_out.push_back(q);
  }
  return 1;
}

bool AffbotIKSolver::getCount(int &count,
                              const int &max_count,
                              const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

int AffbotIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in,
                                    std::vector<KDL::JntArray> &q_out,
                                    const double &timeout)
{
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::WallTime start_time = ros::WallTime::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((affbot_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-affbot_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("positive increments, negative increments: %d %d", num_positive_increments, num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init, p_in, q_out) > 0)
      return 1;
    if(!getCount(count, num_positive_increments, -num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count, q_init(free_angle_));
    loop_time = (ros::WallTime::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds", timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

int AffbotIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in,
                                    KDL::JntArray &q_out,
                                    const double &timeout,
                                    const double& consistency_limit)
{
  moveit_msgs::MoveItErrorCodes error_code;
  static kinematics::KinematicsBase::IKCallbackFn solution_callback = 0;
  return CartToJntSearch(q_in, p_in, q_out, timeout, true, consistency_limit, error_code, solution_callback);
}

int AffbotIKSolver:: CartToJntSearch(const KDL::JntArray& q_in,
                                     const KDL::Frame& p_in,
                                     KDL::JntArray &q_out,
                                     const double &timeout,
                                     moveit_msgs::MoveItErrorCodes &error_code,
                                     const kinematics::KinematicsBase::IKCallbackFn &solution_callback)
{
  return CartToJntSearch(q_in, p_in, q_out, timeout, false, 0.0, error_code, solution_callback);
}


int AffbotIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in,
                                    KDL::JntArray &q_out,
                                    const double &timeout,
                                    const double& consistency_limit,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsBase::IKCallbackFn &solution_callback)
{
  return CartToJntSearch(q_in, p_in, q_out, timeout, true, consistency_limit, error_code, solution_callback);
}

int AffbotIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in,
                                    KDL::JntArray &q_out,
                                    const double &timeout,
                                    bool use_consistency_limit,
                                    const double &max_consistency,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsBase::IKCallbackFn &solution_callback)
{
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::WallTime start_time = ros::WallTime::now();
  double loop_time = 0;
  int count = 0;

  double max_limit, min_limit;
  if(use_consistency_limit)
  {
    max_limit = fmin(affbot_ik_.solver_info_.limits[free_angle_].max_position, initial_guess+max_consistency);
    min_limit = fmax(affbot_ik_.solver_info_.limits[free_angle_].min_position, initial_guess-max_consistency);
  }
  else
  {
    max_limit = affbot_ik_.solver_info_.limits[free_angle_].max_position;
    min_limit = affbot_ik_.solver_info_.limits[free_angle_].min_position;
  }

  int num_positive_increments = (int)((max_limit-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-min_limit)/search_discretization_angle_);

  if(use_consistency_limit)
  {
    ROS_DEBUG("Consistency[Joint: %d]: Initial guess %f, consistency %f", free_angle_, initial_guess, max_consistency);
    ROS_DEBUG("Max limit %f = max(%f, %f)", max_limit, affbot_ik_.solver_info_.limits[free_angle_].max_position, initial_guess+max_consistency);
    ROS_DEBUG("Min limit %f = min(%f, %f)", min_limit, affbot_ik_.solver_info_.limits[free_angle_].min_position, initial_guess-max_consistency);
  }
  else
  {
    ROS_DEBUG("Consistency[Joint: %d]: Initial guess %f", free_angle_, initial_guess);
    ROS_DEBUG("Max limit %f", max_limit);
    ROS_DEBUG("Min limit %f", min_limit);
  }

  ROS_DEBUG("positive increments, negative increments: %d %d", num_positive_increments, num_negative_increments);

  unsigned int testnum = 0;
  geometry_msgs::Pose ik_pose_msg;
  tf::poseKDLToMsg(p_in, ik_pose_msg);

  ros::WallTime s = ros::WallTime::now();

  while(loop_time < timeout)
  {
    testnum++;
    if(CartToJnt(q_init, p_in, q_out) > 0)
    {
      if(solution_callback)
      {
        std::vector<double> ik_solution(7,0.0);
        for(int i=0; i < 7; ++i)
          ik_solution[i] = q_out(i);

        solution_callback(ik_pose_msg, ik_solution, error_code);
        if(error_code.val == error_code.SUCCESS)
        {
          ROS_DEBUG("Difference is %f %f", q_in(free_angle_), q_out(free_angle_));
          ROS_DEBUG("Success with %d in %f", testnum, (ros::WallTime::now()-s).toSec());
          return 1;
        }
      }
      else
      {
        error_code.val = error_code.SUCCESS;
        return 1;
      }
    }
    if(!getCount(count, num_positive_increments, -num_negative_increments))
    {
      ROS_DEBUG("Failure with %d in %f", testnum, (ros::WallTime::now()-s).toSec());
      error_code.val = error_code.NO_IK_SOLUTION;
      return -1;
    }
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("Redundancy search, index:%d, free angle value: %f", count, q_init(free_angle_));
    loop_time = (ros::WallTime::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    error_code.val = error_code.TIMED_OUT;
  }
  else
  {
    error_code.val = error_code.NO_IK_SOLUTION;
  }
  return -1;
}


std::string AffbotIKSolver::getFrameId()
{
  return root_frame_name_;
}
