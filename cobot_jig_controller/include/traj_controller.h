#ifndef TRAJ_CONTROLLER_H
#define TRAJ_CONTROLLER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <sensor_msgs/JointState.h>

#include <boost/shared_ptr.hpp>

#include "cobot_sample_controller/cControl.h"

#define DEFAULT_TRAJECTORY_PLAY_FACTOR 1.0
#define DEFAULT_ANGLE_SAFETY_LIMIT 0.3
#define DEFAULT_GOAL_TOLERANCE 0.05

class TrajectoryController
{
protected:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTActionServerT;
    typedef JTActionServerT::GoalHandle GoalHandle;

public:
    TrajectoryController();
//        const arm_components_name_manager::ArmComponentsNameManager& _joints,
//        std::string& _action_topic_name,
//        std::vector<double>& _targetPos,
//        std::vector<double>& _targetVel,
//        std::vector<double>& _currentAngles,
//        std::vector<double>& _currentVels,
//        boost::mutex& _lock,
//        bool _positionMode,
//        std::vector<double>& _maxVelocities,
//        float _armAngleSafetyLimit = DEFAULT_ANGLE_SAFETY_LIMIT,
//        float _goalTolerance = DEFAULT_GOAL_TOLERANCE,
//        bool _simplifyTrajectoryVelocities = true,
//        bool _useOnlineVelocityControl = true,
//        float _interGoalTolerance = 2 * DEFAULT_GOAL_TOLERANCE);

    ~TrajectoryController();

	cControl control;

	void setHasCurrentGoal(bool flag);
	bool hasCurrentGoal();
	void actionCallback();

	void adaptTrajectoryAngles(trajectory_msgs::JointTrajectory& traj,
														 double diff = 0.02, double epsilon = 1e-03) const;

  bool positionMode;
  bool usePositionMode() const
  {
      return positionMode;
  }

  bool adaptTrajectoryVelocitiesToLinear(trajectory_msgs::JointTrajectory& traj,
												                 const std::vector<int>& joint_indices) const;

  bool checkTrajectory(const trajectory_msgs::JointTrajectory& traj,
								       const std::vector<int>& joint_indices, int group);

  bool atTrajectoryStart(const trajectory_msgs::JointTrajectory& traj,
          const std::vector<int>& joint_indices, int group, float tolerance);

  void updateCurrentState();

  bool removeIrrelevantStates(const std::vector<int>& idx, std::vector<double>& values) const;
  bool getTargetAngles(const trajectory_msgs::JointTrajectoryPoint& p,
          const std::vector<int>& j_idx, std::vector<double>& target) const;
  bool angleDistanceOK(std::vector<double>& j1, std::vector<double>& j2, int check, float maxAngle) const;
  bool getTargetVelocities(const trajectory_msgs::JointTrajectoryPoint& p,
          const std::vector<int>& j_idx, std::vector<double>& target) const;
  bool maxVelExceeded(std::vector<double>& vels, int check, float tolerance_above = 1e-02) const;

  void setTargetValues(const trajectory_msgs::JointTrajectoryPoint& p,
          const std::vector<int>& joint_indices, const int group,
          const std::vector<double>& currentState,
          std::vector<double>& targetState, bool usePositions);

  bool equalJointFloats(const std::vector<double>& first, const std::vector<double>& second,
          float tolerance, bool useMinFingerVal);

  void setExecutionFinished(bool flag, bool success);

  virtual bool playTrajectoryImplementation(const trajectory_msgs::JointTrajectory& traj,
          const std::vector<int>& joint_indices, const int group);

  void playTrajectoryOnlineControlled(const trajectory_msgs::JointTrajectory traj,
                                      const std::vector<int>& joint_indices,
                                      const int group, const float inter_tolerance, const float lagTime);

  bool waitUntilPointReached(const std::vector<double>& _targetPos,
                             const std::vector<double>& _initialTargetVel, float recheckTime,
                             float maxWaitTime, float tolerance, float lagTime,
                             float min_correct_vel, float max_correct_vel);

  bool repeatedWaitUntilPointReached(const std::vector<double>& _targetPos, float recheckTime,
      float maxWaitForExact, float maxWaitForZero, float tolerance, float lagTime,
      float min_correct_vel, float max_correct_vel, int numTries);

  bool goalActive();
  bool waitUntilVelocitiesZero(float recheckTime, float maxWaitTime, float tolerance);

  void maxEndpointDiff(float& maxAngle, int& maxJoint, const trajectory_msgs::JointTrajectory& traj,
          const std::vector<int>& joint_indices, const int group);

  void maxAngleDiff(const std::vector<double>& first, const std::vector<double>& second, float& max, int& idx);

  void playTrajectorySimple(const trajectory_msgs::JointTrajectory traj,
          const std::vector<int>& joint_indices, const int group);

  bool currentTargetReached(const std::vector<double>& targetAngles, float tolerance);
  bool currentTargetReached(const trajectory_msgs::JointTrajectoryPoint& point,
      const std::vector<int>& joint_indices, const int group, float tolerance);

	double sign(double x) const
	{
		return (x > 0.0) - (x < 0.0);
	}

  GoalHandle current_goal;
  boost::mutex goalLock;
  bool has_goal;
  trajectory_msgs::JointTrajectory current_traj;
  std::vector<int> current_traj_idx;
	int current_traj_group;

  bool simplifyTrajectoryVelocities;
  std::vector<double> maxVelocities;

  float ANGLE_SAFETY_LIMIT;
  float GOAL_TOLERANCE;
  float TRAJECTORY_PLAY_FACTOR;
  float INTER_GOAL_TOLERANCE;


	int numTotalJoints = 6;
	int numArmJoints = 6;
	int numGripperJoints = 0;
  bool enableMaxAngleDistSafety;

  bool executionIsFinished;
  bool executionSuccessful;
  bool useOnlineVelocityControl;

  boost::thread * playThread;

  std::vector<double> targetPos;
  std::vector<double> targetVel;

  std::vector<double> current_joint_angles;
  std::vector<double> current_joint_vels;

  bool initialized;

private:
	boost::mutex valueLock;

};

#endif  // TRAJ_CONTROLLER_H
