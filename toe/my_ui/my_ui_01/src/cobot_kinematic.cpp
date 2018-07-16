#ifndef COBOT_KINEMATIC_CPP
#define COBOT_KINEMATIC_CPP

#include <cmath>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include "cobot_kinematic.h"

CobotKinematic::CobotKinematic()
{
	/*
	a(i) = the distance from Z^(i) to Z^(i+1) along X^(i)
	alpha(i) = the angle from Z^(i) to Z^(i+1) along X^(i)
	d(i) = the distance from X^(i-1) to X^(i) along Z^(i)
	theta(i) = the angle from X^(i-1) to X^(i) along Z^(i)

										DH Table
	--------------------------------------------------
		 i   | a(i-1) | alpha(i-1) |   d    | theta
	--------------------------------------------------
		 -   |    0   |     0      |   0    |   0        ** delete this line
	--------------------------------------------------
		 1   |    0   |     0      |   L0   | theta0 			T[0]
	--------------------------------------------------
		 2   |    0   |    -90     |   0    | theta1-90 	T[1] 
	--------------------------------------------------
		 3   |    L1  |     0      |   0    | theta2			T[2]
	--------------------------------------------------
		 4   |    0   |    -90     |   L2   | theta3			T[3]
	--------------------------------------------------
		 5   |    0   |     90     |   0    | theta4+90		T[4]
	--------------------------------------------------
		 6   |    0   |    -90     |   L4   | theta5			T[5]
	--------------------------------------------------
		 7   |    0   |     0      |   L5   |   0					T[6]
	--------------------------------------------------

cos 0 = 1		cos +-pi/2 = 0
sin 0 = 0		sin +-pi/2 = 1,-1

T[i-1, i] = 				cos(theta[i])  				,  			-sin(theta[i])				  ,         0			  	,  				a[i-1]
						sin(theta[i])*cos(alpha[i-1])	,	cos(theta[i])*cos(alpha[i-1])	,	-sin(alpha[i-1])	,	-sin(alpha[i-1])*d[i]
						sin(theta[i])*sin(alpha[i-1])	,	cos(theta[i])*sin(alpha[i-1])	,	 cos(alpha[i-1])	,	 cos(alpha[i-1])*d[i]
												 0								,							 0								,				  0					,					 	1
	*/
	H_PI = M_PI/2.0;
//	std::vector<double> L{0.184, 0.27203, 0.141, 0.109, 0.120, 0.041};
	a = {0, 0, L[1], 0, 0, 0};
	alpha = {0, -H_PI, 0, -H_PI, H_PI, -H_PI};
	d = {L[0], 0, 0, L[2]+L[3], 0, L[4]};
}

std::vector<geometry_msgs::Pose> CobotKinematic::computeFK(std::vector<double> theta)
{
	std::vector<geometry_msgs::Pose> out;
	dh_theta = {theta[0], theta[1]-H_PI, theta[2], theta[3], theta[4]+H_PI, theta[5]};
	std::vector<Eigen::Matrix4f> T;

	for(int i=0; i<dh_theta.size(); i++)
	{
		Eigen::Matrix4f x;
		x << 				cos(dh_theta[i])  				,  			-sin(dh_theta[i])				  ,         0			 	,  				a[i]				,
					sin(dh_theta[i])*cos(alpha[i])	,	cos(dh_theta[i])*cos(alpha[i])	,	-sin(alpha[i])	,	-sin(alpha[i])*d[i]	,
					sin(dh_theta[i])*sin(alpha[i])	,	cos(dh_theta[i])*sin(alpha[i])	,	 cos(alpha[i])	,	 cos(alpha[i])*d[i]	,
												 0								,								 0								,				  0				,					 	1					;
/*		ROS_INFO("-------------t %d--------------", i);
		for(int i=0; i<x.rows(); i++)
			ROS_INFO("%f, %f, %f, %f", x(i, 0), x(i, 1), x(i, 2), x(i, 3));
*/
		T.push_back(x);
	}
	geometry_msgs::Pose point;
	Eigen::Matrix4f t_all = T[0];
	point.position.x = t_all(0, 3);
	point.position.y = t_all(1, 3);
	point.position.z = t_all(2, 3);
	out.push_back(point);
/*	ROS_INFO("-------------t_all 0--------------");
	for(int i=0; i<t_all.rows(); i++)
		ROS_INFO("%f, %f, %f, %f", t_all(i, 0), t_all(i, 1), t_all(i, 2), t_all(i, 3));
*/
	for(int j=1; j<T.size(); j++) {
		t_all = t_all * T[j];
		point.position.x = t_all(0, 3);
		point.position.y = t_all(1, 3);
		point.position.z = t_all(2, 3);
		out.push_back(point);
/*		ROS_INFO("-------------t_all %d--------------", j);
		for(int i=0; i<t_all.rows(); i++)
			ROS_INFO("%f, %f, %f, %f", t_all(i, 0), t_all(i, 1), t_all(i, 2), t_all(i, 3));
*/
	}
	
	return out;
}

std::vector<std::vector<double>> CobotKinematic::computeIK(geometry_msgs::Pose pose)
{
	std::vector<std::vector<double>> solutions;
	std::vector<double> solution;

	ROS_INFO("A1");
	ROS_INFO("%lf, %lf, %lf | %lf, %lf, %lf, %lf", pose.position.x, pose.position.y, pose.position.z
																							 , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	double Px06 = pose.position.x, Py06 = pose.position.y, Pz06 = pose.position.z;

	Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(pose.orientation, q);
	Eigen::Matrix3d r06 = q.normalized().toRotationMatrix();

	ROS_INFO("z06 : ");
	ROS_INFO("[ %lf, %lf, %lf ]", r06(0,0), r06(0,1), r06(0,2));
	ROS_INFO("[ %lf, %lf, %lf ]", r06(1,0), r06(1,1), r06(1,2));
	ROS_INFO("[ %lf, %lf, %lf ]", r06(2,0), r06(2,1), r06(2,2));

	double Px05 = Px06-(L[4]*r06(0,2)), Py05 = Py06-(L[4]*r06(1,2)), Pz05 = Pz06-(L[4]*r06(2,2));
	double Z06x = r06(0,2), Z06y = r06(1,2), Z06z = r06(2,2);
	double Y06x = r06(0,1), Y06y = r06(1,1), Y06z = r06(2,1);

	ROS_INFO("P05 : %lf, %lf, %lf", Px05, Py05, Pz05);
	
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

					ROS_INFO("IK : %lf, %lf, %lf, %lf, %lf, %lf", h1[i1], h2, h3[i3], h4, h5[i5], h6);
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
	return solutions;
}
#endif // COBOT_KINEMATIC_CPP
