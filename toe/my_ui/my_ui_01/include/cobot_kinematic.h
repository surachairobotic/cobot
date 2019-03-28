#ifndef COBOT_KINEMATIC_H
#define COBOT_KINEMATIC_H

#include <vector>
#include <eigen_conversions/eigen_msg.h>

class CobotKinematic
{
public:

std::vector<double> a, alpha, d, dh_theta, theta;
double H_PI;
//std::vector<double> L{0.184, 0.27203, 0.25, 0.109, 0.161};
std::vector<double> L{0.184, 0.27203, 0.25, 0.109, 0.111+0.081};

CobotKinematic();
std::vector<geometry_msgs::Pose> computeFK(std::vector<double> theta);
std::vector<std::vector<double>> computeIK(geometry_msgs::Pose pose);

};

#endif // COBOT_KINEMATIC_H
