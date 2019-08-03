#ifndef __CROTATION_H__
#define __CROTATION_H__

#include <opencv2/opencv.hpp>

// https://www.learnopencv.com/rotation-matrix-to-euler-angles/

// Calculates rotation matrix given euler angles.
void Euler2Matrix(const cv::Vec3f &theta, cv::Mat &R)
{
  // Calculate rotation about x axis
  cv::Mat R_x = (cv::Mat_<double>(3,3) <<
    1,       0,              0,
    0,       cos(theta[0]),   -sin(theta[0]),
    0,       sin(theta[0]),   cos(theta[0])
    );
    
  // Calculate rotation about y axis
  cv::Mat R_y = (cv::Mat_<double>(3,3) <<
    cos(theta[1]),    0,      sin(theta[1]),
    0,               1,      0,
    -sin(theta[1]),   0,      cos(theta[1])
    );
    
  // Calculate rotation about z axis
  cv::Mat R_z = (cv::Mat_<double>(3,3) <<
    cos(theta[2]),    -sin(theta[2]),      0,
    sin(theta[2]),    cos(theta[2]),       0,
    0,               0,                  1);

  // Combined rotation matrix
  R = R_z * R_y * R_x;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(const cv::Mat &R)
{
  cv::Mat Rt;
  cv::transpose(R, Rt);
  cv::Mat shouldBeIdentity = Rt * R;
  cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    
  return  cv::norm(I, shouldBeIdentity) < 1e-6;
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
void Matrix2Euler(const cv::Mat &R, cv::Vec3d &euler)
{ 
  assert(isRotationMatrix(R));
    
  double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

  bool singular = sy < 1e-6; // If

  double x, y, z;
  if (!singular)
  {
    x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
    y = atan2(-R.at<double>(2,0), sy);
    z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }
  else
  {
    x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
    y = atan2(-R.at<double>(2,0), sy);
    z = 0;
  }
  euler = cv::Vec3d(x, y, z);
}

#endif