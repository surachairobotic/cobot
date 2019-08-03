#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cobot_pick/common.h"
#include "cobot_pick/config_find_object.h"
#include "cobot_pick/cConvert3D.h"
#include "cobot_pick/cSegment.h"
#include "cobot_pick/cSelectObject.h"
#include "cobot_pick/cFindLabel.h"
#include "cobot_pick/cROSData.h"

/*sensor_msgs::CameraInfo msg_cam_info;
sensor_msgs::Image msg_depth, msg_col;
sensor_msgs::PointCloud2 msg_pc_org*/

cv_bridge::CvImagePtr p_img_col;
cSegment seg;
cSelectObject select_obj;
cFindLabel find_label;

sensor_msgs::CameraInfo msg_cam_info;
sensor_msgs::Image msg_col, msg_depth;

bool b_cam_info = false, b_col = false, b_depth = false;


void cb_cam_info2(const sensor_msgs::CameraInfo& msg){
  msg_cam_info = msg;
  b_cam_info = true;
}
void cb_depth2(const sensor_msgs::Image& msg){
  msg_depth = msg;
  b_depth = true;
}
void cb_col2(const sensor_msgs::Image& msg){
  msg_col = msg;
  b_col = true;
}


int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ROS_INFO("start");
  ros::init(argc, argv, "cobot_find_object");
  ros::NodeHandle n;

  {
    unsigned int aa[128];
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);

    srand(spec.tv_nsec);
    //    srand((int)ros::Time::now().toNSec());
    for (int i = 0; i < 128; i++)
    {
      aa[i] = (unsigned int)rand();
    }
    InitMtEx(aa, 128);
  }
  
  config.collect_mode = true;
  if( !get_config() )
    return -1;
  
  ros::Subscriber cam_info_sub = n.subscribe("/camera/aligned_depth_to_color/camera_info", 10, cb_cam_info2)
    , depth_sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 10, cb_depth2)
    , col_sub = n.subscribe("/camera/color/image_raw", 10, cb_col2);

  cROSCameraInfo ros_cam_info;
  cROSImage ros_depth, ros_col;
  
  while (ros::ok())
  {
    ros::spinOnce();
    char k = (char)cv::waitKey(30);
    if (k == 27 || k == 'q')
    {
      break;
    }
    if( b_col ){
      p_img_col = cv_bridge::toCvCopy(msg_col, sensor_msgs::image_encodings::BGR8);
      cv::Mat img;
      cv::resize(p_img_col->image, img
        , cv::Size( p_img_col->image.cols/2, p_img_col->image.rows/2 ));
      cv::imshow("col", img);
      if( k=='s' ){
        if( !b_depth ){
          ROS_WARN("no depth");
        }
        else if( !b_cam_info ){
          ROS_WARN("no cam info");
        }
        else{
          ROS_INFO("start process");
          pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
          cConvert3D::convert_pc(msg_depth, msg_cam_info, msg_col, cloud_rgb);
          {
            char fname[256], fname2[256];
            create_filename( config.collect_prefix_data, fname, 0, 0 );
            fname[strlen(fname)-9] = 0;
            
            sprintf(fname2, "%scam_info.bin", fname);
            ros_cam_info.save( msg_cam_info, fname2);
            sprintf(fname2, "%scol.bin", fname);
            ros_col.save( msg_col, fname2);
            sprintf(fname2, "%sdepth.bin", fname);
            ros_depth.save( msg_depth, fname2);
          }
          find_label.run(p_img_col->image, cloud_rgb);
        }
      }
    }
  }
  ROS_INFO("stop");
  return 0;
}
