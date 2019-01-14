
/*
  create scripts/test_eq_c.py to check calculation in eq_c.h with eq.py
*/

#include "cobot_planner/eq_c.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>

double q[6]// = {1.2,3.0,-1.2,0.5,1.9,0.7}
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

void create_test_file(){
    FILE *fp =fopen( "/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/test_eq_c.py", "wt" );
  if( !fp ){
    printf("cannot open file\n");
    return;
  }
  fprintf(fp, "# created by cobot_test_eq.cpp to check eq_c.h calculation\n\nimport numpy as np\nimport eq\n\n");
  
  //////////////////////
  
  fprintf(fp,"q = np.array([");
  for(int i=0;i<6;i++){
      fprintf(fp,"%lf, ", q[i]);
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  
  fprintf(fp,"R = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<4;j++){
      fprintf(fp,"[");
      for(int k=0;k<4;k++){
        fprintf(fp,"%lf, ", R[i][j][k]);
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"dR_dq = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<6;j++){
      fprintf(fp,"[");
      for(int k=0;k<4;k++){
        fprintf(fp,"[");
        for(int l=0;l<4;l++){
          fprintf(fp,"%lf, ", dR_dq[i][j][k][l]);
        }
        fprintf(fp,"],\n");
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"Kw = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<3;j++){
      fprintf(fp,"[");
      for(int k=0;k<6;k++){
        fprintf(fp,"%lf, ", Kw[i][j][k]);
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"dKw_dq = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<6;j++){
      fprintf(fp,"[");
      for(int k=0;k<3;k++){
        fprintf(fp,"[");
        for(int l=0;l<6;l++){
          fprintf(fp,"%lf, ", dKw_dq[i][j][k][l]);
        }
        fprintf(fp,"],\n");
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"J = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<4;j++){
      fprintf(fp,"[");
      for(int k=0;k<6;k++){
        fprintf(fp,"%lf, ", J[i][j][k]);
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"dJ_dq = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<6;j++){
      fprintf(fp,"[");
      for(int k=0;k<4;k++){
        fprintf(fp,"[");
        for(int l=0;l<6;l++){
          fprintf(fp,"%lf, ", dJ_dq[i][j][k][l]);
        }
        fprintf(fp,"],\n");
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"dz_dq = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<6;j++){
      fprintf(fp,"%lf, ", dz_dq[i][j]);
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"M = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"%lf, ", M[i]);
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"I = np.array([");
  for(int i=0;i<6;i++){
    fprintf(fp,"[");
    for(int j=0;j<3;j++){
      fprintf(fp,"[");
      for(int k=0;k<3;k++){
        fprintf(fp,"%lf, ", I[i][j][k]);
      }
      fprintf(fp,"],\n");
    }
    fprintf(fp,"],\n");
  }
  fprintf(fp,"])\n");
  
  ///////////////////////
  
  fprintf(fp,"g = %lf\n\n", g);
  
  std::vector<std::string> names = {"R","dR_dq","Kw","dKw_dq","J"," dJ_dq","dz_dq","M","I","g" };
  
  fprintf(fp, "%s2", names[0].c_str());
  for(int i=1;i<names.size();i++){
    fprintf(fp, ",%s2", names[i].c_str());
  }
  fprintf(fp, " = eq.get_vars(q)\n");
  for(int i=0;i<names.size();i++){
    if( names[i]=="g" ){
      fprintf(fp, "print( \"%s : \" + str(%s - %s2) )\n"
        , names[i].c_str(), names[i].c_str(), names[i].c_str());
    }
    else{
      fprintf(fp, "print( \"%s : \" + str((%s - %s2).max()) )\n"
        , names[i].c_str(), names[i].c_str(), names[i].c_str());
    }
  }
  fclose(fp);

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

/*      m_dR_dq[i][j] = cv::Mat( 3, 3, CV_64F );
      for(int k=0;k<3;k++){
        memcpy( m_dR_dq[i][j].data, &dR_dq[i][j][k][0], sizeof(double)*3 );
      }*/
      m_dR_dq[i][j] = cv::Mat( 6, 6, CV_64F );
      memcpy( m_dR_dq[i][j].data, &dR_dq[i][j][0][0], sizeof(double)*6*6 );
      
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
    , dU_dq[6] = {0.0}
    , dT_ddq[6] = {0.0};
  cv::Mat K_ddq( 6, 6, CV_64F );
  cv::Mat m_dq( 6, 1, CV_64F ), m_ddq( 6, 1, CV_64F );
  memcpy( m_dq.data, dq, sizeof(double)*6 );
  memcpy( m_ddq.data, ddq, sizeof(double)*6 );
  
  for(int i=0;i<6;i++){
    for(int j=0;j<6;j++){
      dT_dq[j]+= 0.5 * ( m_dq.t() * (
        M[i]*( m_dJ_dq[i][j].t() * (m_J[i]) + m_J[i].t() * (m_dJ_dq[i][j]) )
        + m_dKw_dq[i][j].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i])
        + m_Kw[i].t() * (m_dR_dq[i][j]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i])
        + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_dR_dq[i][j].t()) * (m_Kw[i])
        + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_dKw_dq[i][j])
        ) * (m_dq) )<double>(0,0);
      dU_dq[j]+= (M[i]*g*dz_dq[i][j]);

    dT_ddq+= (M[i]*( dJ[i].t() * (m_J[i]) * (m_dq) + m_J[i].t() * (dJ[i]) * (m_dq) )
      + m_dKw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i]) * (m_dq)
      + m_Kw[i].t() * (dR[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i]) * (m_dq)
      + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (dR[i].t()) * (m_Kw[i]) * (m_dq)
      + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_dKw[i]) * (m_dq)).at<double>(0,0);

    K_ddq+= (M[i]*(m_J[i].t() * (m_J[i]))
      + m_Kw[i].t() * (m_R[i]) * (m_I[i]) * (m_R[i].t()) * (m_Kw[i])).at<double>(0,0);
  
  cv::Mat K_ddq2 = K_ddq * (m_ddq);
  for(int i=0;i<6;i++){
    torque_no_ddq[i] = - (dT_dq[i] - dU_dq[i] - dT_ddq[i]);
    torque_ddq[i] = K_ddq2.at<double>(0,0) + torque_no_ddq[i];
  }
}

int main(){


  srand(time(NULL));
  for(int i=0;i<6;i++){
    q[i] = 3.14 * 2 * double(rand() - (RAND_MAX/2)) / RAND_MAX;
  }
  
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
  
  //create_test_file(); 
  
  cv::Mat m_R = cv::Mat( 4, 4, CV_64F );
  double q[6] = {0.0}, dq[6] = {0.0}, ddq[6] = {0.0}
    , torque_no_ddq[6], torque_ddq[6];
  cal_torque( q, dq, ddq, torque_no_ddq, torque_ddq );
  
  
  return 0;
}
