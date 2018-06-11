#include <MyFrame.h>
#include <MyDisplay.h>

#include "ui_multimovedisplay.h"

using namespace my_plugin;

/*
a(i) = the distance from Z^(i) to Z^(i+1) along X^(i)
alpha(i) = the angle from Z^(i) to Z^(i+1) along X^(i)
d(i) = the distance from X^(i-1) to X^(i) along Z^(i)
theta(i) = the angle from X^(i-1) to X^(i) along Z^(i)

									DH Table
--------------------------------------------------
   i   | a(i-1) | alpha(i-1) |   d    | theta
--------------------------------------------------
   0   |    0   |     0      |   0    |   0
--------------------------------------------------
   1   |    0   |     0      |   L1   | theta0 
--------------------------------------------------
   2   |    0   |    -90     |   0    | theta1-90  
--------------------------------------------------
   3   |    L2  |     0      |   0    | theta2
--------------------------------------------------
   4   |    0   |    -90     |   L3   | theta3
--------------------------------------------------
   5   |    0   |    +90     |   0    | theta4+90
--------------------------------------------------
   6   |    0   |    -90     |   L6   | theta5
--------------------------------------------------
   7   |    0   |     0      |   L7   | theta6d
--------------------------------------------------

T[i-1, i] = 				cos(theta[i])  				,  			-sin(theta[i])				  ,         0			  	,  				a[i-1]
						sin(theta[i])*cos(alpha[i-1])	,	cos(theta[i])*cos(alpha[i-1])	,	-sin(alpha[i-1])	,	-sin(alpha[i-1])*d[i]
						sin(theta[i])*sin(alpha[i-1])	,	cos(theta[i])*sin(alpha[i-1])	,	 cos(alpha[i-1])	,	 cos(alpha[i-1])*d[i]
												 0								,							 0								,				  0					,					 	1
*/


