#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cobot_pick/cROSData.h"
#include "cobot_pick/cConvert3D.h"
#include "cobot_pick/cSegment.h"
#include "cobot_pick/cSelectObject.h"
#include <cobot_pick/CobotFindObjectAction.h>
#include <actionlib/server/simple_action_server.h>

/*sensor_msgs::CameraInfo msg_cam_info;
sensor_msgs::Image msg_depth, msg_col;
sensor_msgs::PointCloud2 msg_pc_org*/

sensor_msgs::PointCloud2 msg_pc_seg, msg_pc_plane;
cv_bridge::CvImagePtr p_img_col;
bool b_pub = false;
cSegment seg;
cSelectObject select_obj;


void process_data(){
  b_pub = false;
  ROS_INFO("start process");
  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;

  p_img_col = cv_bridge::toCvCopy(ros_col.msg, sensor_msgs::image_encodings::BGR8);
  cConvert3D::convert_pc(ros_depth.msg, ros_cam_info.msg, ros_col.msg, cloud_rgb);
  pcl::toROSMsg(cloud_rgb, ros_pc.msg);
  seg.seg(cloud_rgb, msg_pc_seg);
  select_obj.run(seg, p_img_col->image);
  pcl::toROSMsg(select_obj.plane_pc, msg_pc_plane);

  ros_pc.msg.header.frame_id = "my_frame";
  msg_pc_seg.header.frame_id = "my_frame";
  msg_pc_plane.header.frame_id = "my_frame";
  b_pub = true;
}


void execute_find_object(const cobot_pick::CobotFindObjectGoalConstPtr &goal
  , actionlib::SimpleActionServer<cobot_pick::CobotFindObjectAction> *as)
{
  cobot_pick::CobotFindObjectFeedback feedback;
  cobot_pick::CobotFindObjectResult result;
  cROSData::start_sub();
  ros::Rate loop_rate(20);
  bool b_ok = false;
  ros::Time t = ros::Time::now();
  for(;;){
    loop_rate.sleep();
    if( cROSData::is_all_saved() ){
      feedback.percent_complete = 0.1;
      as->publishFeedback(feedback);
      process_data();
      for(int i=0;i<select_obj.pick_poses.size();i++){
        result.poses.push_back(select_obj.pick_poses[i].pose);
        result.labels.push_back(select_obj.pick_poses[i].label);
      }
      as->setSucceeded(result);
      return;
    }
    if( !ros::ok() ){
      ROS_WARN("ROS was shutdowned while action is running");
      return;
    }
    if((ros::Time::now()-t).toSec()>10.0){
      ROS_ERROR("Action does not finished within 5 sec.");
      printf("saved : %d, %d, %d\n", (int)ros_cam_info.is_saved()
        , (int)ros_depth.is_saved() 
        , (int)ros_col.is_saved() );
      break;
    }
  }
  
}

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ROS_INFO("start");
  ros::init(argc, argv, "cobot_find_object");
  ros::NodeHandle n;
  cROSData::set_nh(n);

  {
    ros::NodeHandle nh("~");
    std::string str;
    bool b;
    double d;
    int i;
    if (nh.getParam("mode", str))
    {
      nh.deleteParam("mode");
      if (str == "save")
        config.save_mode = true;
      else if (str == "load")
        config.load_mode = true;
      else if (str == "action")
        config.action_server_mode = true;
      else
      {
        ROS_ERROR("Invalid mode : %s", str.c_str());
        return 0;
      }
    }
    if (nh.getParam("save_file_prefix", str))
    {
      nh.deleteParam("save_file_prefix");
      config.save_file_prefix = str;
    }
    if (nh.getParam("result_save_path", str))
    {
      nh.deleteParam("result_save_path");
      config.result_save_path = str;
    }
    if (nh.getParam("show_result", b))
    {
      nh.deleteParam("show_result");
      config.show_result = b;
    }

    if (nh.getParam("normal_th_ang", d))
    {
      nh.deleteParam("normal_th_ang");
      config.normal_th_ang = d;
    }
    if (nh.getParam("th_pointcloud_distance", d))
    {
      nh.deleteParam("th_pointcloud_distance");
      config.th_pointcloud_distance = d;
    }

    if (nh.getParam("normal_win_size", i))
    {
      nh.deleteParam("normal_win_size");
      config.normal_win_size = i;
    }
    if (nh.getParam("normal_thread_num", i))
    {
      nh.deleteParam("normal_thread_num");
      config.normal_thread_num = i;
    }

    // select
    if (nh.getParam("plane_reg_min", i))
    {
      nh.deleteParam("plane_reg_min");
      config.plane_reg_min = i;
    }
    if (nh.getParam("plane_reg_max", i))
    {
      nh.deleteParam("plane_reg_max");
      config.plane_reg_max = i;
    }
    if (nh.getParam("plane_reg_ratio_min", d))
    {
      nh.deleteParam("plane_reg_ratio_min");
      config.plane_reg_ratio_min = d;
    }
    if (nh.getParam("plane_reg_ratio_max", d))
    {
      nh.deleteParam("plane_reg_ratio_max");
      config.plane_reg_ratio_max = d;
    }
    if (nh.getParam("warp_meter2pixel", d))
    {
      nh.deleteParam("warp_meter2pixel");
      config.warp_meter2pixel = d;
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

    if (nh.getParam("text_binarize_win_size", i))
    {
      nh.deleteParam("text_binarize_win_size");
      config.text_binarize_win_size = i;
    }
    if (nh.getParam("text_binarize_subtract", d))
    {
      nh.deleteParam("text_binarize_subtract");
      config.text_binarize_subtract = d;
    }
    if (nh.getParam("text_ransac_repeat_time", i))
    {
      nh.deleteParam("text_ransac_repeat_time");
      config.text_ransac_repeat_time = i;
    }
    if (nh.getParam("text_ransac_th_error", d))
    {
      nh.deleteParam("text_ransac_th_error");
      config.text_ransac_th_error = d;
    }
  }

  ROS_INFO("save : %d", (int)config.save_mode);
  ROS_INFO("load : %d", (int)config.load_mode);
  ROS_INFO("action_server : %d", (int)config.action_server_mode);

  ROS_INFO("show_result : %d", (int)config.show_result);
  ROS_INFO("save_file_prefix : %s", config.save_file_prefix.c_str());
  ROS_INFO("result_save_path : %s", config.result_save_path.c_str());

  ROS_INFO("-- param --");
  ROS_INFO("normal_th_ang : %.3lf", config.normal_th_ang);
  ROS_INFO("th_pointcloud_distance : %.3lf", config.th_pointcloud_distance);
  ROS_INFO("normal_win_size : %d", config.normal_win_size);
  ROS_INFO("normal_thread_num : %d", config.normal_thread_num);

  ROS_INFO("plane_reg_min : %d", config.plane_reg_min);
  ROS_INFO("plane_reg_max : %d", config.plane_reg_max);
  ROS_INFO("plane_reg_ratio_min : %.3lf", config.plane_reg_ratio_min);
  ROS_INFO("plane_reg_ratio_max : %.3lf", config.plane_reg_ratio_max);
  ROS_INFO("warp_meter2pixel : %.3lf", config.warp_meter2pixel);
  ROS_INFO("plane_ransac_repeat_time : %d", config.plane_ransac_repeat_time);
  ROS_INFO("plane_ransac_th_error : %.3lf", config.plane_ransac_th_error);
  ROS_INFO("text_binarize_win_size : %d", config.text_binarize_win_size);
  ROS_INFO("text_binarize_subtract : %.3lf", config.text_binarize_subtract);
  ROS_INFO("text_ransac_repeat_time : %d", config.text_ransac_repeat_time);
  ROS_INFO("text_ransac_th_error : %.3lf", config.text_ransac_th_error);

  /*
  ROS_INFO("norm_th_curvature : %.3lf", config.norm_th_curvature);
  ROS_INFO("segment_radius : %.3lf", config.segment_radius);
  ROS_INFO("min_cluster_size : %d", config.min_cluster_size);
  ROS_INFO("max_cluster_size : %d", config.max_cluster_size);
  */
  /*
  ROS_INFO("reg_min_pix : %d", config.reg_min_pix);
  ROS_INFO("reg_max_pix : %d", config.reg_max_pix);
  ROS_INFO("reg_min_ratio : %.3lf", config.reg_min_ratio);
  ROS_INFO("reg_max_ratio : %.3lf", config.reg_max_ratio);
  ROS_INFO("warp_meter2pixel : %.3lf", config.warp_meter2pixel);
*/
  ros::Rate r(10);
  if (config.save_mode)
  {
    cROSData::save_mode();

    while (ros::ok())
    {
      ros::spinOnce();
      if (cROSData::is_all_saved())
      {
        cROSData::stop_sub();
        ros::shutdown();
        break;
      }
      r.sleep();
    }
  }
  else
  {
    actionlib::SimpleActionServer<cobot_pick::CobotFindObjectAction>
        action_server(n, "/cobot/find_object", boost::bind(&execute_find_object, _1, &action_server), false);

    if (config.load_mode)
    {
      cROSData::load_mode();
      //cROSData::load_mode(msg_cam_info, msg_depth, msg_col, cloud_rgb);
    }
    else if (config.action_server_mode)
    {
      action_server.start();
      printf("start action server\n");
    }

    ros::Publisher pc_pub_org = n.advertise<sensor_msgs::PointCloud2>("/my_pc_org", 20), pc_pub_seg = n.advertise<sensor_msgs::PointCloud2>("/my_pc_seg", 20), pc_pub_plane = n.advertise<sensor_msgs::PointCloud2>("/my_pc_plane", 20), pc_pub_plane_frames = n.advertise<visualization_msgs::Marker>("/my_pc_plane_frames", 20), pc_pub_normal = n.advertise<visualization_msgs::Marker>("/my_pc_normal", 20), pc_pub_pick = n.advertise<visualization_msgs::Marker>("/my_pc_pick", 20);
    while (ros::ok())
    {
      ros::spinOnce();
      if( b_pub ){
        ros_pc.msg.header.stamp = msg_pc_seg.header.stamp = ros::Time::now();
        pc_pub_org.publish(ros_pc.msg);
        pc_pub_seg.publish(msg_pc_seg);
        pc_pub_plane.publish(msg_pc_plane);
        for (int i = select_obj.plane_frames.size() - 1; i >= 0; i--)
          pc_pub_plane_frames.publish(select_obj.plane_frames[i]);
        for (int i = seg.normal_markers.size() - 1; i >= 0; i--)
          pc_pub_normal.publish(seg.normal_markers[i]);
        for (int i = select_obj.pick_markers.size() - 1; i >= 0; i--)
          pc_pub_pick.publish(select_obj.pick_markers[i]);
        if (config.show_result)
        {
          cv::imshow("label", seg.img_col);
          cv::imshow("norm_ang", seg.img_norm_ang);
          cv::imshow("norm1", seg.img_norm1);
        }
      }
      char k = (char)cv::waitKey(30);
      if (k == 27 || k == 'q')
      {
        break;
      }
      r.sleep();
    }
  }
  cROSData::stop_sub();
  ROS_INFO("stop");
/*  while(!ros::isShuttingDown()){
    r.sleep();
  }*/
  return 0;
}
