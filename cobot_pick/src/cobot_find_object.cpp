#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "sensor_msgs/Image.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

#include "cobot_pick/cROSData.h"
#include "cobot_pick/convert_pc.h"

using namespace pcl;
using namespace std;

cv::Mat img;
ros::Publisher pc_pub;
image_geometry::PinholeCameraModel model;


void seg(pcl::PointCloud<pcl::PointXYZRGB> &cloud_rgb, sensor_msgs::PointCloud2 &msg);


/*

void cam_info_callback(const sensor_msgs::CameraInfo& msg)
{
  if( b_save && !b_end_info ){
    cROSCameraInfo::save( (save_prefix+"_cam_info.bin").c_str(), msg);
    b_end_info = true;
  }
}

void cam_info_callback(const sensor_msgs::CameraInfo& msg)
{
  if( b_save && !b_end_info ){
    cROSCameraInfo::save( (save_prefix+"_cam_info.bin").c_str(), msg);
    b_end_info = true;
  }
}
*/
void pc_callback(const sensor_msgs::PointCloud2& msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg( msg, cloud );
/*  if( b_save ){
    if( !b_end_pc ){
      pcl::io::savePCDFileASCII (save_prefix + "_pc.pcd", cloud);
      b_end_pc = true;
    }
    return;
  }*/

  sensor_msgs::PointCloud2 msg2;
  seg(cloud, msg2);
  pc_pub.publish(msg2);
}

/*
void load(pcl::PointCloud<pcl::PointXYZRGB> &cloud){
  pcl::io::loadPCDFile(save_prefix+"_pc.pcd", cloud);
}*/


void convert_rgb2xyz(pcl::PointCloud<pcl::PointXYZRGB> &cloud_rgb
    , pcl::PointCloud<pcl::PointXYZ> &cloud){
  
  cloud.width = 640;
  cloud.height = int(cloud_rgb.width / cloud.width);
  cloud.resize(cloud.width*cloud.height);
  for(int i=cloud.size()-1;i>=0;i--){
    cloud.points[i].x = cloud_rgb.points[i].x;
    cloud.points[i].y = cloud_rgb.points[i].y;
    cloud.points[i].z = cloud_rgb.points[i].z;
  }
}

template<typename PointT>
void reorganize(pcl::PointCloud<PointT> &cloud){
  const int shift = 213, width = 640;
  const float e = 0.00001;
  int cnt = 0;
  double min_z = 9999999, max_z = -9999999;
  std::vector<PointT, Eigen::aligned_allocator<PointT> > ps(cloud.points);
  cloud.width = width;
  cloud.height = cloud.points.size() / width;
  ROS_INFO("point size : %d / %d, same = %d", (int)cloud.points.size()
    , (int)(cloud.width * cloud.height)
    , int(cloud.points.size()==(cloud.width * cloud.height)));
  for(int i=cloud.height-1;i>=0;i--){
    const int i2 = i*cloud.width;
    for(int j=cloud.width-1;j>=0;j--){
      const PointT &p1 = ps[i2+j];
      PointT &p2 = cloud.points[i2+(j+width-shift)%shift];
      if(fabs(p1.x)<e && fabs(p1.y)<e && fabs(p1.z)<e ){
        p2.x = p2.y = p2.z = std::numeric_limits<float>::quiet_NaN(); 
      }
      else{
        p2 = p1;
        if( p1.z>max_z)
          max_z = p1.z;
        if( p1.z<min_z)
          min_z = p1.z;
        cnt++;
      }
    }
  }
  ROS_INFO("cnt valid : %d", cnt);
  ROS_INFO("min max z : %.3lf, %.3lf", min_z, max_z);
  cloud.is_dense = false;
}

void seg(pcl::PointCloud<pcl::PointXYZRGB> &cloud_rgb, sensor_msgs::PointCloud2 &msg){
//  pcl::PointCloud<pcl::PointXYZ> cloud;

  reorganize(cloud_rgb);
  pcl::toROSMsg(cloud_rgb, msg);
  return;

//  convert_rgb2xyz(cloud_rgb, cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>());
/*  if( cloud_rgb.points.size()>20000){
    cloud_rgb.points.resize(20000);
    cloud_rgb.width = cloud_rgb.points.size();
  }*/
  cloud = cloud_rgb.makeShared();
  ROS_INFO("width : %d, height : %d", cloud->width, cloud->height);

  // Create a search tree, use KDTreee for non-organized data.
  double scale1 = 0.005, scale2 = 0.01, threshold = 0.5, segradius = 0.01;
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  ROS_INFO("cloud organized : %d", (int)cloud->isOrganized ());
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
  }
  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);
  if (scale1 >= scale2)
  {
    ROS_ERROR("Error: Large scale must be > small scale!");
    return;
  }

  // Compute normals using both small and large scales at each point
//  pcl::NormalEstimation<PointXYZRGB, PointNormal> ne;
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  cout << "Compute ..." << endl;
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    return;
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
/*  if( f_don.size()>0 ){
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> (f_don, *doncloud, false);
  }*/

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
    new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
  );

  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);
  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
//  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (1);
  ec.setMaxClusterSize (50);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  ROS_INFO("cluster num : %d", (int)cluster_indices.size());
  uint8_t col[][3] = {{255,0,0},{0,255,0},{0,0,255},
    {255,255,0},{255,0,255},{0,255,255},
    {150,0,0},{0,150,0},{0,0,150},
    {150,150,0},{150,0,150},{0,150,150}
  };
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc2( new pcl::PointCloud<pcl::PointXYZRGB>());
  PointXYZRGB pnt;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    //pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    const uint8_t *c = &col[j%12][0];
    pnt.r = c[0];
    pnt.g = c[1];
    pnt.b = c[2];
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
    //  cloud_cluster_don->points.push_back (doncloud->points[*pit]);
      const PointNormal &p = doncloud->points[*pit];
      pnt.x = p.x;
      pnt.y = p.y;
      pnt.z = p.z;
      pc2->points.push_back(pnt);
    }
    /*
    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;
    */
  }
  pc2->width = pc2->points.size();
  pc2->height = 1;
  pc2->is_dense = true;

  pcl::toROSMsg(*pc2, msg);


/*  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr( new pcl::PointCloud<pcl::PointXYZ>());
  
  cloudPtr = cloud.makeShared(); 
  seg.setInputCloud (cloudPtr);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }
  
  ROS_INFO("inliers indices : %d\n", inliers->indices.size ());

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
                                               << cloud.points[inliers->indices[i]].y << " "
                                               << cloud.points[inliers->indices[i]].z << std::endl;
  */
}

ros::Subscriber ss;
void cb_ss(const sensor_msgs::Image& msg){}

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ROS_INFO("start");
  ros::init(argc, argv, "cobot_find_object");
  ros::NodeHandle n;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
  sensor_msgs::PointCloud2 msg;
  bool b_save = false, b_load = false;
  {
    ros::NodeHandle nh("~");
    std::string str;
    bool b;
    if( nh.getParam("save", b) ){
      nh.deleteParam("save");
      b_save = b;
    }
    if( nh.getParam("load", b) ){
      nh.deleteParam("load");
      b_load = b;
    }
    if( nh.getParam("file_prefix", str) ){
      nh.deleteParam("file_prefix");
      cROSData::file_prefix = str;
    }
    
  }

  ROS_INFO("save : %d", (int)b_save);
  ROS_INFO("load : %d", (int)b_load);
  ROS_INFO("file_prefix : %s", cROSData::file_prefix.c_str());

  if( b_save ){
    cROSData::save_mode(n);
  }
  else if( b_load ){
    sensor_msgs::CameraInfo msg_cam_info;
    sensor_msgs::Image msg_depth;
    cROSData::load_mode(msg_cam_info, msg_depth, cloud_rgb);
//      seg(cloud_rgb, msg);
    msg.header.frame_id = "world";
  }
  return 0;


//  img = cv::Mat(640,480,CV_8UC3);
  pc_pub = n.advertise<sensor_msgs::PointCloud2>("/seg", 20);
//  ros::Subscriber pc_sub;
  /*
  if( b_save || !b_load ){
    pc_sub = n.subscribe("/camera/depth/color/points", 10, pc_callback);
    cam_info_sub = n.subscribe("/camera/aligned_depth_to_color/camera_info", 10, cam_info_callback);
    depth_sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 10, depth_callback);
  }*/
  ros::Rate r(10);
  
  
  while (ros::ok()){
    ros::spinOnce();
//    cv::imshow("PC", img);
//    cv::waitKey(100);
    if( cROSData::is_all_saved() ){
      cROSData::stop();
      ros::shutdown();
      break;
    }
    msg.header.stamp = ros::Time::now();
    pc_pub.publish(msg);
    r.sleep();
  }
  while( false == ros::isShuttingDown() ){
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
