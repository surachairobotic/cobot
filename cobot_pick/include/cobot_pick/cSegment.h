
#ifndef __CSEGMENT_H__
#define __CSEGMENT_H__

#include "cobot_pick/common.h"
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ros/ros.h"

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include "cobot_pick/cLabeling.h"
#include "cobot_pick/cNormal.h"



class cSegment{

private:
  cLabeling label;

  bool cal_normal(){

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized ())
      tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
    else
      tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));

    // Set the input pointcloud for the search tree
    tree->setInputCloud (cloud);
    if (norm_scale1 >= norm_scale2){
      ROS_ERROR("Error: Large scale must be > small scale!");
      return false;
    }

    // Compute normals using both small and large scales at each point
  //  pcl::NormalEstimation<PointXYZRGB, PointNormal> ne;
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
//    cout << "Calculating normals for scale..." << norm_scale1 << endl;
//    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

    if( norm_k_search1>0 )
      ne.setKSearch(norm_k_search1);
    else
      ne.setRadiusSearch (norm_scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
//    cout << "Calculating normals for scale..." << norm_scale2 << endl;
//    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

    if( norm_k_search2>0 )
      ne.setKSearch(norm_k_search2);
    else
      ne.setRadiusSearch (norm_scale2);
    ne.compute (*normals_large_scale);

    return true;
  }

  bool cal_normal_my(){
    CalNormal3_with_TBB( cloud, normal_my, norm_k_search_my, norm_thread);
    return true;
  }


  void filter_normal(){
    
    //const pcl::PointNormal *n1 = &normals_small_scale->points[0]
    //  , *n2 = &normals_large_scale->points[0]
    //  , **ns[2] = {&n1, &n2};
    const double th_cos = cos(norm_th_rad);
    img_bin = cv::Mat(cloud->height,cloud->width,CV_8UC1);
    img_label = cv::Mat(cloud->height,cloud->width,CV_16UC1);
    img_col = cv::Mat(cloud->height,cloud->width,CV_8UC3);
    img_norm_curve1 = cv::Mat(cloud->height,cloud->width,CV_8UC1);
    img_norm_curve2 = cv::Mat(cloud->height,cloud->width,CV_8UC1);
    img_norm_ang = cv::Mat(cloud->height,cloud->width,CV_8UC1);
    img_norm1 = cv::Mat(cloud->height,cloud->width,CV_8UC1);
    img_norm2 = cv::Mat(cloud->height,cloud->width,CV_8UC1);
    uint8_t *p = (uint8_t*)img_bin.data
      , *p_c1 = (uint8_t*)img_norm_curve1.data
      , *p_c2 = (uint8_t*)img_norm_curve2.data
      , *p_a = (uint8_t*)img_norm_ang.data
      , *p_n[2] = {(uint8_t*)img_norm1.data, (uint8_t*)img_norm2.data};

    const double TH_DIS2 = POW2(threshold_pointcloud_distance);
    memset( p, 0, img_bin.rows*img_bin.cols);
    
    int cnt = 0;
    for(int i=img_bin.rows-2;i>=0;i--){
      const int i2 = i*img_bin.cols;
      for(int j=img_bin.cols-2;j>=0;j--){
        const int j2 = i2+j;
        const pcl::PointXYZRGB &p1 = cloud->points[j2]
          , &p2 = cloud->points[j2+1], &p3 = cloud->points[j2+img_bin.cols];
        if( IS_VALID_POINT(p1) && DIS2( p1, p2 )<TH_DIS2 && DIS2( p1, p3 )<TH_DIS2 ){
          p[j2] = 255;
        }
        else{
          cnt++;
        }
      }
    }
    ROS_INFO("cnt : %d", cnt);
    label.ExecBin(img_bin, img_label);
    label.CreateImageResult( img_label, img_col, true);
/*
    for(int i=0;i<cloud->points.size();i++,n1++,n2++,p++){
      double norm_cos = fabs(n1->normal_x*n2->normal_x 
          + n1->normal_y*n2->normal_y + n1->normal_z*n2->normal_z);
      if( norm_cos>1.0 )
        norm_cos = 1.0;

      *p = uint8_t(( n1->curvature < norm_th_curvature 
          || n2->curvature < norm_th_curvature
          || norm_cos < th_cos
        ) ? 0 : 255);
      if( n1->curvature > 1.0 || n1->curvature<0.0
        || n2->curvature > 1.0 || n2->curvature<0.0 ){
        ROS_INFO("Invalid curve ang : %.3lf, %.3lf, %.3lf"
          , n1->curvature, n2->curvature, norm_cos);
        ROS_INFO("n1 : %.3lf, %.3lf, %.3lf", n1->normal_x, n1->normal_y, n1->normal_z);
        ROS_INFO("n2 : %.3lf, %.3lf, %.3lf", n2->normal_x, n2->normal_y, n2->normal_z);
      }
      else{
        p_c1[i] = n1->curvature*255;
        p_c2[i] = n2->curvature*255;

        double v = fabs(acos(norm_cos))*255.0 * 10.0/( M_PI ); // max = 18 deg
        p_a[i] = v>255.0 ? 255.0 : v;

        for(int j=0;j<2;j++){
          double a = fabs((*ns)[j]->normal_z);
          if( a>1.0 ) a = 1.0;
          a = fabs(acos(a))*255.0 * 2.0/( M_PI ); // max = 45 deg
          p_n[j][i] = a>255.0 ? 255.0 : a;
        }
      }
    }*/

//    label.ExecBin(img_bin, img_label);
//    label.CreateImageResult( img_label, img_col, true);


  }

public:
  double norm_scale1, norm_scale2, norm_th_rad, norm_th_curvature, segment_radius
    , threshold_pointcloud_distance;
  int min_cluster_size, max_cluster_size, norm_k_search1, norm_k_search2
    , norm_k_search_my, norm_thread;
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale, normals_large_scale, normal_my;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  cv::Mat img_label, img_bin, img_col, img_norm_curve1, img_norm_curve2, img_norm_ang
    , img_norm1, img_norm2;


  cSegment():norm_scale1(0.007), norm_scale2(0.015)
    , norm_k_search1(0), norm_k_search2(0)
    , norm_th_rad(0.1), threshold_pointcloud_distance(0.01)
    , norm_th_curvature(0.9),segment_radius(0.01)
    , norm_k_search_my(4), norm_thread(1)
    , min_cluster_size(10), max_cluster_size(50000)
    , cloud( new pcl::PointCloud<pcl::PointXYZRGB>())
    , normals_small_scale (new pcl::PointCloud<pcl::PointNormal>)
    , normals_large_scale (new pcl::PointCloud<pcl::PointNormal>)
    , normal_my (new pcl::PointCloud<pcl::PointNormal>)
    {}


  void seg(pcl::PointCloud<pcl::PointXYZRGB> &cloud_rgb, sensor_msgs::PointCloud2 &msg){
    cloud = cloud_rgb.makeShared();
    ROS_INFO("width : %d, height : %d", cloud->width, cloud->height);

    ros::Time t = ros::Time::now();
//    cal_normal();
    cal_normal_my();
    printf("xx : %lf", cloud->points[0].x);
    ROS_INFO("cal_normal() : %.3lf s", (ros::Time::now()-t).toSec());
    t = ros::Time::now();
    filter_normal();
    ROS_INFO("filter_normal() : %.3lf s", (ros::Time::now()-t).toSec());
  }
};

#endif