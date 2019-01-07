#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cobot_pick/common.h"
#include "cobot_pick/config_calibrate.h"
#include "cobot_pick/zdsfmt.h"
#include "cobot_pick/cROSData.h"
#include "cobot_pick/cConvert3D.h"

std::vector<visualization_msgs::Marker> markers;


bool find_corners(const cv::Mat &src, std::vector<cv::Point2f> &corners){
  cv::Mat img_gray = cv::Mat( src.size(), CV_8UC1 )
    , img_pat = src.clone();
  cv::Size chess_num(config.chess_num[0], config.chess_num[1]);
  cv::cvtColor( src, img_gray, cv::COLOR_BGR2GRAY);
  bool b_found = cv::findChessboardCorners( img_gray
    , chess_num
    , corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
        + cv::CALIB_CB_FAST_CHECK );
  if( !b_found ){
    ROS_WARN("pat not found");
    return false;
  }
  cv::cornerSubPix( img_gray, corners
    , cv::Size(11, 11), cv::Size(-1, -1)
    , cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.01));

  // rearrange corners
  {
    const int x = config.chess_num[0], y = config.chess_num[1];
    std::vector<cv::Point2f> new_corners(corners.size());
    if( corners[0].x < corners[1].x ){
      if( corners[0].y < corners[config.chess_num[0]].y ){
        // do nothing
      }
      else{
        for(int i=0;i<y;i++){
          for(int j=0;j<x;j++){
            new_corners[i*x + j] = corners[(y-i-1)*x + j];
          }
        }
        corners = new_corners;
      }
    }
    else{
      if( corners[0].y < corners[config.chess_num[0]].y ){
        for(int i=0;i<y;i++){
          for(int j=0;j<x;j++){
            new_corners[i*x + (x-j-1)] = corners[i*x + j];
          }
        }
      }
      else{
        for(int i=0;i<y;i++){
          for(int j=0;j<x;j++){
            new_corners[(y-i-1)*x + (x-j-1)] = corners[i*x + j];
          }
        }
      }
      corners = new_corners;
    }
    assert( corners[0].x < corners[1].x &&
      corners[0].y < corners[config.chess_num[0]].y );
  }

  if( config.show_result ){
    cv::drawChessboardCorners(img_pat, chess_num, cv::Mat(corners), true);
    cv::imshow("pat", img_pat);
  }

}



inline bool check_in_reg(int x,int y, const cv::Point2f *reg_corners){
  for(int i=3;i>=0;i--){
    const cv::Point2f &p1 = reg_corners[i], &p2 = reg_corners[(i+1)%4]
      , v1( x-p1.x, y-p1.y), v2( p2.x-p1.x, p2.y-p1.y);
    if( v1.x*v2.y - v1.y*v2.x > 0.0 )
      return false;
  }
  return true;
}



class cPlane
{
public:
  pcl::PointXYZ point, normal, x_axis, y_axis, center;
  cPlane(){}

  void set_plane(const pcl::PointXYZRGB &_p, const pcl::PointXYZ &_normal, const pcl::PointXYZ &_x_axis)
  {
    point.x = _p.x;
    point.y = _p.y;
    point.z = _p.z;
    normal = _normal;
    NORMALIZE_3D(x_axis, _x_axis);
    CROSS_3D(y_axis, normal, x_axis);
  }
};

void find_plane(const pcl::PointCloud<pcl::PointXYZRGB> &cloud
    , const std::vector<cv::Point2f> &corners
    , std::vector<pcl::PointXYZ> &corners_3d
    ){
  
  // find reg
  const float PAD_CORNER = 5;
  cv::Point2f reg_corners[4], reg[2];
  reg_corners[0].x = corners[0].x - PAD_CORNER;
  reg_corners[0].y = corners[0].y - PAD_CORNER;
  reg_corners[1].x = corners[config.chess_num[0]-1].x + PAD_CORNER;
  reg_corners[1].y = corners[config.chess_num[0]-1].y - PAD_CORNER;
  reg_corners[2].x = corners[corners.size()-1].x + PAD_CORNER;
  reg_corners[2].y = corners[corners.size()-1].y + PAD_CORNER;
  reg_corners[3].x = corners[corners.size()-config.chess_num[0]].x - PAD_CORNER;
  reg_corners[3].y = corners[corners.size()-config.chess_num[0]].y + PAD_CORNER;

  reg[0].x = reg[0].y = 999999;
  reg[1].x = reg[1].y = -999999;
/*  for(int i=0;i<4;i++){
    if( reg[0].x<reg_corners[i].x )
      reg[0].x = reg_corners[i].x;
    if( reg[0].y<reg_corners[i].y )
      reg[0].y = reg_corners[i].y;
    
    if( reg[1].x>reg_corners[i].x )
      reg[1].x = reg_corners[i].x;
    if( reg[1].y>reg_corners[i].y )
      reg[1].y = reg_corners[i].y;
  }*/
  
  reg[0].x = reg_corners[0].x < reg_corners[3].x ? reg_corners[0].x : reg_corners[3].x;
  reg[0].y = reg_corners[0].y < reg_corners[1].y ? reg_corners[0].y : reg_corners[1].y;
  reg[1].x = reg_corners[1].x > reg_corners[2].x ? reg_corners[1].x : reg_corners[2].x;
  reg[1].y = reg_corners[2].y > reg_corners[3].y ? reg_corners[2].y : reg_corners[3].y;
/*
  for(int i=0;i<4;i++){
    const cv::Point2f &p1 = reg_corners[i], &p2 = reg_corners[(i+1)%4];
    printf("dis : %.3f\n", POW2(p1.x-p2.x) + POW2(p1.y-p2.y));
  }*/

  struct tPoint2i
  {
    int i, j, u, v;
    const pcl::PointXYZRGB *p;
    tPoint2i() {}
    tPoint2i(int _i, int _j, pcl::PointXYZRGB *_p) : i(_i), j(_j), p(_p) {}
  };

  std::vector<tPoint2i> vec_pix;
  vec_pix.reserve(cloud.height * cloud.width / 2);
  {
    tPoint2i p;
    cv::Mat img_reg = cv::Mat( cloud.height, cloud.width, CV_8UC1, cv::Scalar(0) );
    for(int i=(int)reg[0].y;i<=(int)reg[1].y;i++){
      const int i2 = i*cloud.width;
      for(int j=(int)reg[0].x;j<=(int)reg[1].x;j++){
        if( IS_VALID_POINT(cloud.points[i2+j]) && check_in_reg(j,i,reg_corners) ){
          p.i = i;
          p.j = j;
          p.p = &cloud.points[i2+j];
          vec_pix.push_back( p );
          img_reg.data[i2+j] = 255;
        }
      }
    }
    if( config.show_result ){
      cv::imshow("reg", img_reg);
    }
  }

  // ransac
  cPlane plane;
  {
    const int MIN_DIS_BETWEEN_POINTS2 = POW2(0.02);
    int best_cnt = 0;
    for (int k1 = config.plane_ransac_repeat_time - 1; k1 >= 0; k1--)
    {
      const tPoint2i *ps[3];
      // randomize 3 points to create plane
      {
        int cnt2 = 5;
        bool b_ok2;
        do
        {
          b_ok2 = true;
          ps[0] = &vec_pix[NextMt() % vec_pix.size()];
          for (int i = 1; i < 3; i++)
          {
            bool b_ok;
            int cnt = 50;
            do
            {
              b_ok = true;
              ps[i] = &vec_pix[NextMt() % vec_pix.size()];
              for (int j = 0; j < i; j++)
              {
                if (DIS2(*(ps[i]->p), *(ps[j]->p)) < MIN_DIS_BETWEEN_POINTS2)
                {
                  b_ok = false;
                  break;
                }
              }
            } while (!b_ok && --cnt >= 0);
            if (!b_ok)
            {
              assert(--cnt2 >= 0);
              b_ok2 = false;
              break;
            }
          }
        } while (!b_ok2);
      }

      assert(DIS2(*(ps[0]->p), *(ps[1]->p)) >= MIN_DIS_BETWEEN_POINTS2 && DIS2(*(ps[0]->p), *(ps[2]->p)) >= MIN_DIS_BETWEEN_POINTS2 && DIS2(*(ps[1]->p), *(ps[2]->p)) >= MIN_DIS_BETWEEN_POINTS2);

      // find plane
      {
        pcl::PointXYZ v[4], cross;
        MINUS_3D(v[0], *(ps[1]->p), *(ps[0]->p));
        MINUS_3D(v[1], *(ps[2]->p), *(ps[0]->p));
        NORMALIZE_3D(v[0]);
        NORMALIZE_3D(v[1]);
        CROSS_3D(cross, v[0], v[1]);
        double len2 = LEN2(cross);
        if (len2 < 0.01)
        {
          k1++;
          continue;
        }
        else
        {
          double len = (cross.z < 0.0 ? -1.0 : 1.0) / sqrt(len2);
          cross.x *= len;
          cross.y *= len;
          cross.z *= len;
        }

        int cnt = 0;
        pcl::PointXYZ vtmp;
        // count points near the plane
        for (int i = vec_pix.size() - 1; i >= 0; i--)
        {
          MINUS_3D(vtmp, *vec_pix[i].p, *ps[0]->p);
          double dis = fabs(INNER_3D(vtmp, cross));
          if (dis < config.plane_ransac_th_error)
          {
            cnt++;
          }
        }
        if (cnt > best_cnt)
        {
          plane.set_plane(*ps[0]->p, cross, v[0]);
          best_cnt = cnt;
        }
      }
    }
  }

  corners_3d.resize(corners.size());
  visualization_msgs::Marker marker;
  geometry_msgs::Point marker_p;
  marker.header.frame_id = "my_frame";
  marker.ns = "corners";
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
  marker.color.r = 0.2;
  marker.color.g = 0.2;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.id = 0;

  const double np = INNER_3D(plane.normal, plane.point);
  for(int i=0;i<corners.size();i++){
    pcl::PointXYZ &p1 = corners_3d[i];//, p2;// = corners_3d[i];
    cConvert3D::get_vector(corners[i].x, corners[i].y, p1);

    // find corner 3d points on the plane
    const double k = np / INNER_3D(plane.normal, p1);
    p1.x *= k;
    p1.y *= k;
    p1.z *= k;

    // check if the point is really on the plane
    {
      pcl::PointXYZ p2;
      MINUS_3D(p2, p1, plane.point);
      assert(fabs(INNER_3D(plane.normal, p2)) < 0.000001);
    }

    if( config.show_result ){
      marker_p.x = corners_3d[i].x;
      marker_p.y = corners_3d[i].y;
      marker_p.z = corners_3d[i].z;
      marker.points.push_back(marker_p);
    }
  }
  // check size
  {
    for(int i=0;i<config.chess_num[1]-1;i++){
      for(int j=0;j<config.chess_num[0]-1;j++){
        const pcl::PointXYZ &p0 = corners_3d[i*config.chess_num[0]+j]
          , &p1 = corners_3d[i*config.chess_num[0]+j+1]
          , &p2 = corners_3d[(i+1)*config.chess_num[0]+j];
        const double d1 = sqrt(DIS2(p0, p1)), d2 = sqrt(DIS2(p0, p2));
        assert( fabs(d1 - config.chess_size[0])<0.002 );
        assert( fabs(d2 - config.chess_size[0])<0.002 );
      }
    }
  }
  if( config.show_result ){
    markers.push_back(marker);
  }
}


void find_translation_matrix(const std::vector<pcl::PointXYZ> &corners_3d
  , geometry_msgs::TransformStamped &transform){
//  , tf::Transform &transform){
  pcl::PointXYZ vx, vy;
  vx.x = (config.chess_pos[1][0] - config.chess_pos[0][0]) / (config.chess_num[0]-1);
  vx.y = (config.chess_pos[1][1] - config.chess_pos[0][1]) / (config.chess_num[0]-1);
  vx.z = (config.chess_pos[1][2] - config.chess_pos[0][2]) / (config.chess_num[0]-1);

  vy.x = (config.chess_pos[3][0] - config.chess_pos[0][0]) / (config.chess_num[1]-1);
  vy.y = (config.chess_pos[3][1] - config.chess_pos[0][1]) / (config.chess_num[1]-1);
  vy.z = (config.chess_pos[3][2] - config.chess_pos[0][2]) / (config.chess_num[1]-1);

  //std::vector<pcl::PointXYZ> real_corners;
  
  double sum_src[3] = {0.0}, sum_dst[3] = {0.0};
  cv::Mat svd_src = cv::Mat( corners_3d.size(), 3, CV_64F)
    , svd_dst = cv::Mat( corners_3d.size(), 3, CV_64F);
  for(int i=0;i<config.chess_num[1];i++){
    for(int j=0;j<config.chess_num[0];j++){
      pcl::PointXYZ p;
      p.x = vx.x * j + vy.x * i + config.chess_pos[0][0];
      p.y = vx.y * j + vy.y * i + config.chess_pos[0][1];
      p.z = vx.z * j + vy.z * i + config.chess_pos[0][2];
//      real_corners.push_back(p);
      int j2 = i*config.chess_num[0]+j;

      sum_dst[0]+= p.x;
      sum_dst[1]+= p.y;
      sum_dst[2]+= p.z;
      svd_dst.at<double>(j2,0) = p.x;
      svd_dst.at<double>(j2,1) = p.y;
      svd_dst.at<double>(j2,2) = p.z;
      svd_src.at<double>(j2,0) = corners_3d[j2].x;
      svd_src.at<double>(j2,1) = corners_3d[j2].y;
      svd_src.at<double>(j2,2) = corners_3d[j2].z;
      sum_src[0]+= corners_3d[j2].x;
      sum_src[1]+= corners_3d[j2].y;
      sum_src[2]+= corners_3d[j2].z;
    }
  }

  // check size
  {
    for(int i=0;i<config.chess_num[1]-1;i++){
      for(int j=0;j<config.chess_num[0]-1;j++){
        const int j2 = i*config.chess_num[0]+j;
        const pcl::PointXYZ p0 = pcl::PointXYZ(
          svd_dst.at<double>(j2,0),
          svd_dst.at<double>(j2,1),
          svd_dst.at<double>(j2,2)
        ),
        p1 = pcl::PointXYZ(
          svd_dst.at<double>(j2+1,0),
          svd_dst.at<double>(j2+1,1),
          svd_dst.at<double>(j2+1,2)
        ),
        p2 = pcl::PointXYZ(
          svd_dst.at<double>(j2+config.chess_num[0],0),
          svd_dst.at<double>(j2+config.chess_num[0],1),
          svd_dst.at<double>(j2+config.chess_num[0],2)
        );
        const double d1 = sqrt(DIS2(p0, p1)), d2 = sqrt(DIS2(p0, p2));
        assert( fabs(d1 - config.chess_size[0])<0.005 );
        assert( fabs(d2 - config.chess_size[0])<0.005 );
      }
    }
  }

  cv::Mat mean_src = cv::Mat( 3, 1, CV_64F)
    , mean_dst = cv::Mat( 3, 1, CV_64F);
  for(int i=0;i<3;i++){
    mean_src.at<double>(i,0) = sum_src[i] / corners_3d.size();
    mean_dst.at<double>(i,0) = sum_dst[i] / corners_3d.size();
  }

  cv::Mat svd_src_norm = svd_src.clone()
    , svd_dst_norm = svd_dst.clone();
  for(int i=corners_3d.size()-1;i>=0;i--){
    for(int j=2;j>=0;j--){
      svd_src_norm.at<double>(i,j)-= mean_src.at<double>(j,0);
      svd_dst_norm.at<double>(i,j)-= mean_dst.at<double>(j,0);
    }
  }
  cv::Mat H = svd_src_norm.t() * svd_dst_norm;
  cv::SVD svd( H, cv::SVD::Flags::MODIFY_A);
  cv::Mat R = svd.vt.t() * svd.u.t();
  if( cv::determinant(R) < 0 ){
    svd.vt.at<double>(2,0) *= -1.0;
    svd.vt.at<double>(2,1) *= -1.0;
    svd.vt.at<double>(2,2) *= -1.0;
    R = svd.vt.t() * svd.u.t();
  }

  cv::Mat T = -R*mean_src + mean_dst;
  
  // save to files
  {
    cv::Mat T_rep, dst2, err;
    cv::repeat( T, 1, corners_3d.size(), T_rep);
    dst2 = R*svd_src.t() + T_rep;
    err = dst2.t() - svd_dst;

    std::ofstream f;
    f.open ("/home/tong/catkin_ws/src/cobot/cobot_pick/calib.txt");
    f << "R : " << R << std::endl;
    f << "T : " << T << std::endl;
    f << "H : " << H << std::endl;
    f << "svd.u : " << svd.u << std::endl;
    f << "svd.vt : " << svd.vt << std::endl;
    f << "src : " << svd_src << std::endl;
    f << "dst : " << svd_dst << std::endl;
    f << "dst2: " << dst2.t() << std::endl;
    f << "err : " << err << std::endl;
    f.close();

    // err
    {
      double sum = 0.0, max = 0.0;
      for(int i=corners_3d.size()-1;i>=0;i--){
        for(int j=2;j>=0;j--){
          double d = err.at<double>(i,j);
          sum+= d;
          if( d>max )
            max = d;
        }
      }
      printf("err : mean = %lf, max = %lf\n", sum/corners_3d.size(), max);
    }
  }

  // tf
  {
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(R.at<double>(0,0))
      , static_cast<double>(R.at<double>(0,1))
      , static_cast<double>(R.at<double>(0,2)) 
      , static_cast<double>(R.at<double>(1,0))
      , static_cast<double>(R.at<double>(1,1))
      , static_cast<double>(R.at<double>(1,2))
      , static_cast<double>(R.at<double>(2,0))
      , static_cast<double>(R.at<double>(2,1))
      , static_cast<double>(R.at<double>(2,2))); /*
    tf3d.setValue(static_cast<double>(R.at<double>(0,0))
      , static_cast<double>(R.at<double>(1,0))
      , static_cast<double>(R.at<double>(2,0)) 
      , static_cast<double>(R.at<double>(0,1))
      , static_cast<double>(R.at<double>(1,1))
      , static_cast<double>(R.at<double>(2,1))
      , static_cast<double>(R.at<double>(0,2))
      , static_cast<double>(R.at<double>(1,2))
      , static_cast<double>(R.at<double>(2,2)));*/
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    /*
    tf2::Vector3 origin;
    
    origin.setValue(static_cast<double>(T.at<double>(0,0))
      ,static_cast<double>(T.at<double>(1,0))
      ,static_cast<double>(T.at<double>(2,0)));
      
    origin.setValue(static_cast<double>(-T.at<double>(0,0))
      ,static_cast<double>(-T.at<double>(1,0))
      ,static_cast<double>(-T.at<double>(2,0)));
*/
    //transform.setOrigin(origin);
    //transform.setRotation(tfqt);
    transform.transform.translation.x = T.at<double>(0,0);
    transform.transform.translation.y = T.at<double>(1,0);
    transform.transform.translation.z = T.at<double>(2,0);
    transform.transform.rotation.x = tfqt.x();
    transform.transform.rotation.y = tfqt.y();
    transform.transform.rotation.z = tfqt.z();
    transform.transform.rotation.w = tfqt.w();

    printf("trans : %lf %lf %lf\n", transform.transform.translation.x
      , transform.transform.translation.y
      , transform.transform.translation.z);
    printf("rotate: %lf %lf %lf %lf\n", transform.transform.rotation.x
      , transform.transform.rotation.y
      , transform.transform.rotation.z
      , transform.transform.rotation.w);
  }
  
}


void create_chess_pos(){
  const double org[3] = {0.1, 0.1, 0.1}; //{1.2, 2.3, 0.2};
  const int N[4][2] = { { 0, 0}, { config.chess_num[0] - 1, 0}
    , { config.chess_num[0] - 1, config.chess_num[1]-1 }
    , { 0, config.chess_num[1]-1 }};

  for(int i=0;i<4;i++){
    config.chess_pos[i][0] = org[0] + N[i][1]*config.chess_size[1];
    config.chess_pos[i][1] = org[1] + N[i][0]*config.chess_size[0];
    config.chess_pos[i][2] = org[2];
  }
}


int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ROS_INFO("start");
  ros::init(argc, argv, "cobot_pick_calibrate");
  ros::NodeHandle n;
  cROSData::set_nh(n);

  {
    unsigned int aa[128];
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    
    srand(spec.tv_nsec);
    for (int i = 0; i < 128; i++)
    {
      aa[i] = (unsigned int)rand();
    }
    InitMtEx(aa, 128);
  }

  {
    ros::NodeHandle nh("~");
    std::string str;
    bool b;
    double d;
    int i;

    if (nh.getParam("load_mode", b))
    {
      nh.deleteParam("load_mode");
      config.load_mode = b;
    }
    if (nh.getParam("save_file_prefix", str))
    {
      nh.deleteParam("save_file_prefix");
      config.save_file_prefix = str;
    }
    if (nh.getParam("show_result", b))
    {
      nh.deleteParam("show_result");
      config.show_result = b;
    }

    if (nh.getParam("chess_num_x", i))
    {
      nh.deleteParam("chess_num_x");
      config.chess_num[0] = i;
    }
    if (nh.getParam("chess_num_y", i))
    {
      nh.deleteParam("chess_num_y");
      config.chess_num[1] = i;
    }    


    if (nh.getParam("chess_size_x", d))
    {
      nh.deleteParam("chess_size_x");
      config.chess_size[0] = d;
    }
    if (nh.getParam("chess_size_y", d))
    {
      nh.deleteParam("chess_size_y");
      config.chess_size[1] = d;
    }

    const char xyz[] = "xyz";
    char s[16];
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
        sprintf(s, "chess_pos%d_%c", i, xyz[j]);
        if (nh.getParam(s, d)){
          nh.deleteParam(s);
          config.chess_pos[i][j] = d;
        }
      }
    }

    if (nh.getParam("plane_ransac_repeat_time", i))
    {
      nh.deleteParam("plane_ransac_repeat_time");
      config.plane_ransac_repeat_time = i;
    }
    if (nh.getParam("plane_ransac_th_error", d))
    {
      nh.deleteParam("plane_ransac_th_error");
      config.plane_ransac_th_error = d;
    }
    
  }
  create_chess_pos();

  ROS_INFO("load : %d", (int)config.load_mode);
  ROS_INFO("show_result : %d", (int)config.show_result);
  ROS_INFO("save_file_prefix : %s", config.save_file_prefix.c_str());

  ROS_INFO("-- param --");
  ROS_INFO("chess_num : %d, %d", config.chess_num[0], config.chess_num[1]);
  ROS_INFO("chess_size : %.3lf, %.3lf", config.chess_size[0], config.chess_size[1]);
  for(int i=0;i<4;i++){
    ROS_INFO("chess_pos%d : %.3lf, %.3lf, %.3lf", i
    , config.chess_pos[i][0], config.chess_pos[i][1], config.chess_pos[i][2]);
  }
  ROS_INFO("plane_ransac_repeat_time : %d", config.plane_ransac_repeat_time);
  ROS_INFO("plane_ransac_th_error : %.3lf", config.plane_ransac_th_error);

  ros::Rate r(10);
  if (config.load_mode)
  {
    cROSData::load_mode();
  }
  else{
    cROSData::start_sub();
    ros::Rate loop_rate(20);
    bool b_ok = false;
    ros::Time t = ros::Time::now();
    do{
      loop_rate.sleep();
      if( !ros::ok() ){
        ROS_WARN("ROS was shutdowned while subscribing data");
        return 0;
      }
      if((ros::Time::now()-t).toSec()>10.0){
        ROS_ERROR("Action does not finished within 5 sec.");
        printf("saved : %d, %d, %d\n", (int)ros_cam_info.is_saved()
          , (int)ros_depth.is_saved() 
          , (int)ros_col.is_saved() );
        return -1;
      }
    }while(!cROSData::is_all_saved());
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
  cv_bridge::CvImagePtr p_img_col;
  p_img_col = cv_bridge::toCvCopy(ros_col.msg, sensor_msgs::image_encodings::BGR8);
  cConvert3D::convert_pc(ros_depth.msg, ros_cam_info.msg, ros_col.msg, cloud_rgb);
  if( config.show_result ){
    pcl::toROSMsg(cloud_rgb, ros_pc.msg);
    ros_pc.msg.header.frame_id = "my_frame";
  }

  std::vector<cv::Point2f> corners;
  std::vector<pcl::PointXYZ> corners_3d;
  geometry_msgs::TransformStamped transform;
  //tf::Transform transform;
  if( find_corners( p_img_col->image, corners ) ){
    find_plane( cloud_rgb, corners, corners_3d);
    find_translation_matrix(corners_3d, transform);
  }


  ros::Publisher pub_markers = n.advertise<visualization_msgs::Marker>("/calib_markers", 20)
    , pub_pc = n.advertise<sensor_msgs::PointCloud2>("/my_pc_org", 20);
//  tf::TransformBroadcaster br;
  tf2_ros::StaticTransformBroadcaster br;

  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "my_frame";
  br.sendTransform(transform);
  while (ros::ok())
  {
    ros::spinOnce();
    if (config.show_result)
    {
      cv::imshow("col", p_img_col->image);
      for (int i = markers.size() - 1; i >= 0; i--){
        markers[i].header.stamp = ros::Time::now();
        pub_markers.publish(markers[i]);
      }

      ros_pc.msg.header.stamp = ros::Time::now();
      pub_pc.publish(ros_pc.msg);

      //br.sendTransform(tf::StampedTransform( transform, ros::Time::now()
      //  , "world", "my_frame"));
    }

    char k = (char)cv::waitKey(10);
    if (k == 27 || k == 'q')
    {
      break;
    }
    r.sleep();
  }

  cROSData::stop_sub();
  ROS_INFO("stop");
/*  while(!ros::isShuttingDown()){
    r.sleep();
  }*/
  return 0;
}
