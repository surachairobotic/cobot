
/*
  create scripts/test_eq_c.py to check calculation in eq_c.h with eq.py
*/

#include "cobot_planner/eq_c.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <sys/time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>

double q[6], dq[6], ddq[6]
  , R[6][4][4]
  , dR_dq[6][6][4][4]
  , Kw[6][3][6]
  , dKw_dq[6][6][3][6]
  , J[6][4][6]
  , dJ_dq[6][6][4][6]
  , dz_dq[6][6]
  , M[6]
  , I[6][3][3]
  , g;

double torque_no_ddq[6], torque_ddq[6];



const char fname_save[] = "/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/test_eq_c_param.py";
bool b_first_save = true;


void fprintf_array( const double *param, const char *name, int d1=1, int d2=1, int d3=1, int d4=1 ){
  FILE *fp;
  if( b_first_save ){
    fp =fopen( fname_save, "wt" );    
    fprintf(fp, "# created by cobot_test_eq.cpp to check eq_c.h calculation\n\nimport numpy as np\nimport eq\n\n");
    b_first_save = false;
  }
  else{
    fp =fopen( fname_save, "at" );   
  }
  if( !fp ){
    printf("cannot open file : %s\n", fname_save);
    exit(-1);
  }  
  
  if( d1==1 ){
    fprintf(fp,"%s = %lf\n\n", name, *param);
    return;
  }
  fprintf(fp,"%s = np.array([", name);
  for(int i=0;i<d1;i++){
    if( d2>1 ){
      fprintf(fp,"[");
      for(int j=0;j<d2;j++){
        if( d3>1 ){
          fprintf(fp,"[");
          for(int k=0;k<d3;k++){
            if( d4>1 ){
              fprintf(fp,"[");
              for(int l=0;l<d4;l++){
                fprintf(fp,"%lf, ", param[((i*d2 + j)*d3 + k)*d4 + l]);
              }
              fprintf(fp,"],\n");
            }
            else{
              fprintf(fp,"%lf, ", param[(i*d2 + j)*d3 + k]);
            }
          }
          fprintf(fp,"],\n");
        }
        else{
          fprintf(fp,"%lf, ", param[i*d2 + j]);
        }
      }
      fprintf(fp,"],\n");
    }
    else{
      fprintf(fp,"%lf, ", param[i]);
    }
  }
  fprintf(fp,"])\n");
  fclose(fp);
}

void fprintf_array( const cv::Mat *mat, const char *name, int d1=1, int d2=1 ){
  double arr[6*6*6*6*6];
  for(int i=0;i<d1;i++){
    for(int j=0;j<d2;j++){
      int n = i*d2 + j;
      const cv::Mat &m = mat[n];
      for(int k1=0;k1<m.rows;k1++){
        for(int k2=0;k2<m.cols;k2++){
          arr[ (n*m.rows + k1)*m.cols + k2] = m.at<double>(k1,k2);
        }
      }
    }
  }
  if( d1==1 )
    fprintf_array( arr, name, mat[0].rows, mat[0].cols );
  else if( d2==1 )
    fprintf_array( arr, name, d1, mat[0].rows, mat[0].cols );
  else
    fprintf_array( arr, name, d1, d2, mat[0].rows, mat[0].cols );
}


void create_test_file(){
  
  fprintf_array( q, "q", 6 );
  fprintf_array( dq, "dq", 6 );
  fprintf_array( ddq, "ddq", 6 );
  fprintf_array( (double*)R, "R", 6, 4, 4 );
  fprintf_array( (double*)dR_dq, "dR_dq", 6, 6, 4, 4 );
  fprintf_array( (double*)Kw, "Kw", 6, 3, 6 );
  fprintf_array( (double*)dKw_dq, "dKw_dq", 6, 6, 3, 6 );
  fprintf_array( (double*)J, "J", 6, 4, 6 );
  fprintf_array( (double*)dJ_dq, "dJ_dq", 6, 6, 4, 6 );
  fprintf_array( (double*)dz_dq, "dz_dq", 6, 6 );
  fprintf_array( M, "M", 6 );
  fprintf_array( (double*)I, "I", 6, 3, 3 );
  fprintf_array( &g, "g" );
  fprintf_array( torque_no_ddq, "torque_no_ddq", 6 );
  fprintf_array( torque_ddq, "torque_ddq", 6 );

}


void cal_torque(const double q[6],const double dq[6],const double ddq[6]
    , double torque_no_ddq[6], double torque_ddq[6] ){
  cv::Mat dR[6], dKw[6], dJ[6];
  cv::Mat m_dJ_dq[6][6], m_J[6], m_dR_dq[6][6], m_dKw_dq[6][6]
    , m_R[6], m_I[6], m_Kw[6];
  for(int i=0;i<6;i++){
    for(int j=0;j<6;j++){
      m_dJ_dq[i][j] = cv::Mat( 4, 6, CV_64F );
      memcpy( m_dJ_dq[i][j].data, &dJ_dq[i][j][0][0], sizeof(double)*4*6 );

      m_dR_dq[i][j] = cv::Mat( 4, 4, CV_64F );
      memcpy( m_dR_dq[i][j].data, &dR_dq[i][j][0][0], sizeof(double)*4*4 );
      
      m_dKw_dq[i][j] = cv::Mat( 3, 6, CV_64F );
      memcpy( m_dKw_dq[i][j].data, &dKw_dq[i][j][0][0], sizeof(double)*3*6 );
    }
    m_J[i] = cv::Mat( 4, 6, CV_64F );
    memcpy( m_J[i].data, &J[i][0][0], sizeof(double)*4*6 );

    m_R[i] = cv::Mat( 3, 3, CV_64F );
    for(int j=0;j<3;j++)
      memcpy( m_R[i].data + sizeof(double)*3*j, &R[i][j][0], sizeof(double)*3 );
      
    m_I[i] = cv::Mat( 3, 3, CV_64F );
    memcpy( m_I[i].data, &I[i][0][0], sizeof(double)*3*3 );

    m_Kw[i] = cv::Mat( 3, 6, CV_64F );
    memcpy( m_Kw[i].data, &Kw[i][0][0], sizeof(double)*3*6 );
  }
  
  cv::Rect roi(0,0,3,3);
  for(int i=0;i<6;i++){
    cv::Mat &dr = dR[i], &dk = dKw[i], &dj = dJ[i];
    for(int j=0;j<6;j++){
      if( j==0 ){
        dr = m_dR_dq[i][j]*dq[j];
        dj = m_dJ_dq[i][j]*dq[j];
        dk = m_dKw_dq[i][j]*dq[j];
      }
      else{
        dr+= m_dR_dq[i][j]*dq[j];
        dj+= m_dJ_dq[i][j]*dq[j];
        dk+= m_dKw_dq[i][j]*dq[j];
      }
      m_dR_dq[i][j] = m_dR_dq[i][j](roi);
    }
    dr = dr(roi);
  }
  

  double dT_dq[6] = {0.0}
    , dU_dq[6] = {0.0};
  cv::Mat dT_ddq( 6, 1, CV_64F, cv::Scalar(0) );
  cv::Mat K_ddq( 6, 6, CV_64F, cv::Scalar(0) );
  cv::Mat m_dq( 6, 1, CV_64F ), m_ddq( 6, 1, CV_64F );
  memcpy( m_dq.data, dq, sizeof(double)*6 );
  memcpy( m_ddq.data, ddq, sizeof(double)*6 );
  
  for(int i=0;i<6;i++){
    for(int j=0;j<6;j++){
      dT_dq[j]+= 0.5 * cv::Mat( m_dq.t() * (
        M[i]*( m_dJ_dq[i][j].t() * (m_J[i]) + m_J[i].t() * (m_dJ_dq[i][j]) )
        + m_dKw_dq[i][j].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i])
        + m_Kw[i].t() * (m_dR_dq[i][j]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i])
        + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_dR_dq[i][j].t()) * (m_Kw[i])
        + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_dKw_dq[i][j])
        ) * (m_dq) ).at<double>(0,0);
      dU_dq[j]+= (M[i]*g*dz_dq[i][j]);
    }

    dT_ddq+= M[i]*( dJ[i].t() * (m_J[i]) * (m_dq) + m_J[i].t() * (dJ[i]) * (m_dq) )
      + dKw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i]) * (m_dq)
      + m_Kw[i].t() * (dR[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i]) * (m_dq)
      + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (dR[i].t()) * (m_Kw[i]) * (m_dq)
      + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (dKw[i]) * (m_dq);
    K_ddq+= M[i]*(m_J[i].t() * (m_J[i]))
      + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i]);
  }
  cv::Mat K_ddq2 = K_ddq * (m_ddq);
  for(int i=0;i<6;i++){
    torque_no_ddq[i] = - (dT_dq[i] - dU_dq[i] - dT_ddq.at<double>(i,0) );
    torque_ddq[i] = K_ddq2.at<double>(i,0) + torque_no_ddq[i];
  }
  
  // for debug
  {
    fprintf_array( (cv::Mat*)m_dJ_dq, "m_dJ_dq", 6, 6 );
    fprintf_array( (cv::Mat*)m_J, "m_J", 6 );
    fprintf_array( (cv::Mat*)m_dR_dq, "m_dR_dq", 6, 6 );
    fprintf_array( (cv::Mat*)m_dKw_dq, "m_dKw_dq", 6, 6 );
    fprintf_array( (cv::Mat*)m_R, "m_R", 6 );
    fprintf_array( (cv::Mat*)m_I, "m_I", 6 );
    fprintf_array( (cv::Mat*)m_Kw, "m_Kw", 6 );
    
    fprintf_array( dT_dq, "dT_dq", 6 );
    fprintf_array( dU_dq, "dU_dq", 6 );
    fprintf_array( &dT_ddq, "dT_ddq", 1 );
    fprintf_array( &K_ddq, "K_ddq", 1 );
    fprintf_array( &m_dq, "m_dq", 1 );
    fprintf_array( &m_ddq, "m_ddq", 1 );
  }
}

int main(){


  struct timeval t1, t2;
  gettimeofday(&t1,NULL);
  srand(t1.tv_usec);
  for(int i=0;i<6;i++){
    q[i] = 3.14 * 2 * double(rand() - (RAND_MAX/2)) / RAND_MAX;
    dq[i] = 3.14 * 2 * double(rand() - (RAND_MAX/2)) / RAND_MAX;
    ddq[i] = 3.14 * 2 * double(rand() - (RAND_MAX/2)) / RAND_MAX;
  }
/*  for(int i=0;i<6;i++){
    q[i] = 0.2*i;
    dq[i] = 0.3*i;
    ddq[i] = 0.4*i;
  }*/
  gettimeofday(&t1,NULL);

  get_R( q, R );
  get_dR_dq( q, dR_dq );
  get_Kw( q, Kw );
  get_dKw_dq( q, dKw_dq );
  get_J( q, J );
  get_dJ_dq( q, dJ_dq );
  get_dz_dq( q, dz_dq );
  get_M( M );
  get_I( I );
  get_g( g );
  
  cal_torque( q, dq, ddq, torque_no_ddq, torque_ddq );
  
  gettimeofday(&t2,NULL);
  double elapsed = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec)*1.0e-6;
  printf("time : %.5lf [s]\n",elapsed);
  
  create_test_file();
  return 0;
}
