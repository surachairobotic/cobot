
/*
  create scripts/test_eq_c.py to check calculation in eq_c.h with eq.py
*/

#include "cobot_planner/eq_c.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <time.h>
#include <stdlib.h>

int main(){

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
  
  FILE *fp =fopen( "/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/test_eq_c.py", "wt" );
  if( !fp ){
    printf("cannot open file\n");
    return -1;
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
  return 0;
}
