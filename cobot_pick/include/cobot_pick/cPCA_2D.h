#ifndef __CPCA_2D_H__
#define __CPCA_2D_H__

#include "cobot_pick/common.h"
#include <opencv2/opencv.hpp>
#include <math.h>

class cPCA_2D{
public:
  double nx,ny,xx,xy,yy;
  double mean[2];
  int n;
  CvMat *var, *eig, *eig_v;

  cPCA_2D(){
    var = cvCreateMat(2,2, CV_64F);
		eig = cvCreateMat(2,1, CV_64F);
		eig_v = cvCreateMat(2, 2, CV_64F);
  }
  ~cPCA_2D(){
    cvReleaseMat(&var);
    cvReleaseMat(&eig);
    cvReleaseMat(&eig_v);
  }
  void reset(){
    nx = ny = xx = xy = yy = 0.0;
    n = 0;
  }

  void run(){
    
    double dtmp;
    mean[0] = nx/n;
    mean[1] = ny/n;
    dtmp = 1.0/(n-1);
    
    {
      double *v = var->data.db;
      v[0] = (xx - n*mean[0]*mean[0])*dtmp;
      v[3] = (yy - n*mean[1]*mean[1])*dtmp;
      v[1] = v[2] = (xy - n*mean[0]*mean[1])*dtmp;
    }
    cvEigenVV (var, eig_v, eig, 0.00000001);
  //	cvSVD(var, eig, eig_v, 0, CV_SVD_U_T + CV_SVD_MODIFY_A);
    assert( eig->data.db[0] >= eig->data.db[1] );
/*    pcl::PointNormal &np = normals[i*cloud->width + j];
    if( eig_v->data.db[8]>0.0 ){
      np.normal_x = eig_v->data.db[2];
      np.normal_y = eig_v->data.db[5];
      np.normal_z = eig_v->data.db[8];
    }
    else{
      np.normal_x = -eig_v->data.db[2];
      np.normal_y = -eig_v->data.db[5];
      np.normal_z = -eig_v->data.db[8];
    }*/
  }
};

#endif