#ifndef __CONFIG_FIND_OBJECT_H__
#define __CONFIG_FIND_OBJECT_H__

#include <string>
#include <tf/tf.h>
#include "ros/ros.h"
#include <chrono>
#include <ctime>
#include <cstdlib>





bool create_dir(const char *path){
  char str[256];
  sprintf(str, "mkdir -p %s", path);
  int dir_err = system(str);
  if (-1 == dir_err){
    printf("Error creating directory : %s\n", path);
    return false;
  }
  return true;
}

struct tConfig{

  bool save_mode, load_mode, action_server_mode, show_result, save_result, collect_mode;
  std::string save_file_prefix, result_save_path, collect_save_path, data_save_path
    , collect_prefix_label, collect_prefix_data, collect_prefix_label_raw;

  int object_type;

  // segment
  double normal_th_ang, th_pointcloud_distance;
    //, norm_scale1, norm_scale2
    //, norm_th_curvature, segment_radius
    
  int normal_win_size, normal_thread_num;

  // select object
  int plane_reg_min, plane_reg_max, plane_ransac_repeat_time, text_binarize_win_size
    , text_ransac_repeat_time, th_region_color
    , min_label_pix_num, max_label_pix_num, min_label_size_x, min_label_size_y, label_mean_rgb, find_plane_radius;
  double plane_reg_ratio_min, plane_reg_ratio_max, warp_meter2pixel
    , plane_ransac_th_error, text_binarize_subtract, text_ransac_th_error
    , min_label_ratio, max_label_ratio, dis_remove_edge, label_multiply_rgb, pc_dis2plane
    , box_size_x, box_size_y, box_size_z, box_label_shift_x
    , basket_size_x, basket_size_y, basket_size_z
    , label_width, label_height, label_size_error;
    
  tf::Transform tf_cam;

  tConfig():
    save_mode(false), load_mode(false), show_result(false), collect_mode(false)
    , action_server_mode(false), save_result(false)
    , data_save_path("/home/tong/catkin_ws/src/cobot/cobot_pick/data/")
    , save_file_prefix("")
    , result_save_path("/home/tong/catkin_ws/src/cobot/cobot_pick/results")
    , object_type( OBJECT_BOX )

    // segment
    , normal_th_ang(0.1)
    , th_pointcloud_distance(0.01)
    , normal_win_size(4)
    , normal_thread_num(1)

    // select object
    , plane_reg_min(0)
    , plane_reg_max(99999999)
    , plane_reg_ratio_min(1.0)
    , plane_reg_ratio_max(1000000.0)
    , warp_meter2pixel(2000.0)
    , plane_ransac_repeat_time(100)
    , plane_ransac_th_error(0.01)
    , text_binarize_win_size(120)
    , text_binarize_subtract(20.0)
    , text_ransac_repeat_time(200)
    , text_ransac_th_error(2)
    , th_region_color(500)
    
    , min_label_pix_num(250)
    , max_label_pix_num(2000)
    , min_label_ratio(1.0)
    , max_label_ratio(3.0)
    , dis_remove_edge(2.0)
    , min_label_size_x(10)
    , min_label_size_y(10)
    , label_mean_rgb(140)
    , label_multiply_rgb(3.0)
    , pc_dis2plane(0.005)
    , find_plane_radius(150)
    
    , box_size_x(0.10)
    , box_size_y(0.07)
    , box_size_z(0.03)
    , box_label_shift_x(0.03)
    
    , basket_size_x(0.17)
    , basket_size_y(0.11)
    , basket_size_z(0.05)

    , label_width(0.022)
    , label_height(0.012)
    , label_size_error(0.004)
    {}
} config;


bool get_config(){
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
      return false;
    }
  }
  if (nh.getParam("object_type", str))
  {
    nh.deleteParam("object_type");
    if (str == "box")
      config.object_type = OBJECT_BOX;
    else if (str == "basket")
      config.object_type = OBJECT_BASKET;
    else
    {
      ROS_ERROR("Invalid object_type : %s", str.c_str());
      return false;
    }
  }
  if (nh.getParam("save_file_prefix", str))
  {
    nh.deleteParam("save_file_prefix");
    config.save_file_prefix = str;
  }
  if (nh.getParam("data_save_path", str))
  {
    nh.deleteParam("data_save_path");
    config.data_save_path = str;
  }
  if (nh.getParam("result_save_path", str))
  {
    nh.deleteParam("result_save_path");
    config.result_save_path = str;
  }
  if (nh.getParam("collect_save_path", str))
  {
    nh.deleteParam("collect_save_path");
    config.collect_save_path = str;
  }
  if (nh.getParam("show_result", b))
  {
    nh.deleteParam("show_result");
    config.show_result = b;
  }
  if (nh.getParam("save_result", b))
  {
    nh.deleteParam("save_result");
    config.save_result = b;
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
  if (nh.getParam("th_region_color", i))
  {
    nh.deleteParam("th_region_color");
    config.th_region_color = i;
  }
  if (nh.getParam("min_label_pix_num", i))
  {
    nh.deleteParam("min_label_pix_num");
    config.min_label_pix_num = i;
  }
  if (nh.getParam("max_label_pix_num", i))
  {
    nh.deleteParam("max_label_pix_num");
    config.max_label_pix_num = i;
  }
  if (nh.getParam("min_label_ratio", d))
  {
    nh.deleteParam("min_label_ratio");
    config.min_label_ratio = d;
  }
  if (nh.getParam("max_label_ratio", d))
  {
    nh.deleteParam("max_label_ratio");
    config.max_label_ratio = d;
  }
  if (nh.getParam("dis_remove_edge", d))
  {
    nh.deleteParam("dis_remove_edge");
    config.dis_remove_edge = d;
  }
  if (nh.getParam("min_label_size_x", i))
  {
    nh.deleteParam("min_label_size_x");
    config.min_label_size_x = i;
  }
  if (nh.getParam("min_label_size_y", i))
  {
    nh.deleteParam("min_label_size_y");
    config.min_label_size_y = i;
  }
  if (nh.getParam("label_mean_rgb", i))
  {
    nh.deleteParam("label_mean_rgb");
    config.label_mean_rgb = i;
  }
  if (nh.getParam("label_multiply_rgb", d))
  {
    nh.deleteParam("label_multiply_rgb");
    config.label_multiply_rgb = d;
  }
  if (nh.getParam("pc_dis2plane", d))
  {
    nh.deleteParam("pc_dis2plane");
    config.pc_dis2plane = d;
  }
  if (nh.getParam("find_plane_radius", i))
  {
    nh.deleteParam("find_plane_radius");
    config.find_plane_radius = i;
  }
  
  if (nh.getParam("box_size_x", d))
  {
    nh.deleteParam("box_size_x");
    config.box_size_x = d;
  }
  if (nh.getParam("box_size_y", d))
  {
    nh.deleteParam("box_size_y");
    config.box_size_y = d;
  }
  if (nh.getParam("box_size_z", d))
  {
    nh.deleteParam("box_size_z");
    config.box_size_z = d;
  }
  if (nh.getParam("box_label_shift_x", d))
  {
    nh.deleteParam("box_label_shift_x");
    config.box_label_shift_x = d;
  }
  
  
  if (nh.getParam("basket_size_x", d))
  {
    nh.deleteParam("basket_size_x");
    config.basket_size_x = d;
  }
  if (nh.getParam("basket_size_y", d))
  {
    nh.deleteParam("basket_size_y");
    config.basket_size_y = d;
  }
  if (nh.getParam("basket_size_z", d))
  {
    nh.deleteParam("basket_size_z");
    config.basket_size_z = d;
  }
  if (nh.getParam("label_width", d))
  {
    nh.deleteParam("label_width");
    config.label_width = d;
  }
  if (nh.getParam("label_height", d))
  {
    nh.deleteParam("label_height");
    config.label_height = d;
  }
  if (nh.getParam("label_size_error", d))
  {
    nh.deleteParam("label_size_error");
    config.label_size_error = d;
  }
  
  if (nh.getParam("tf_world2camera", str))
  {
    nh.deleteParam("tf_world2camera");
    int i1 = 0, n = str.size(), cnt = 0;
    double dtmp[7];
    std::string str_org = str;
    for (int i = 0; i <= n; i++)
    {
      const char c = str[i];
      if (c == ' ' || i == n)
      {
        if (i > i1)
        {
          if (cnt >= 7)
          {
            ROS_ERROR("tf_world2camera data num is larger than 7 : %s", str_org.c_str());
            return false;
          }
          str[i] = 0;
          std::string s = str.c_str() + i1;
          dtmp[cnt++] = std::stod(s);
          i1 = i + 1;
        }
      }
      else if (!((c >= '0' && c <= '9') || c == '-' || c == '.' || c == 'e' || c == 'E'))
      {
        ROS_ERROR("invalid char found in tf_world2camera : %c / %s", c, str_org.c_str());
        return false;
      }
    }
    if (cnt != 7)
    {
      ROS_ERROR("tf_world2camera data num is not 7 : %s", str_org.c_str());
      return false;
    }
    d = sqrt(POW2(dtmp[3]) + POW2(dtmp[4]) + POW2(dtmp[5]) + POW2(dtmp[6]));
    if (fabs(d - 1.0) > 0.001)
    {
      ROS_ERROR("tf_world2camera invalid quat : %s", str_org.c_str());
      assert(0);
      return false;
    }

    config.tf_cam.setOrigin(tf::Vector3(dtmp[0], dtmp[1], dtmp[2]));
    config.tf_cam.setRotation(tf::Quaternion(dtmp[3], dtmp[4], dtmp[5], dtmp[6]));
  }

  // cannot show image in action thread
  if (config.action_server_mode)
  {
    config.show_result = false;
  }
  
  if( config.collect_mode ){
    std::time_t t = std::time(0);   // get time now
    std::tm* now = std::localtime(&t);
    
    config.collect_prefix_label = config.collect_save_path + "label/";
    config.collect_prefix_label_raw = config.collect_save_path + "label_raw/";
    config.collect_prefix_data = config.collect_save_path + "data/";
    
    std::string* dir[3] = { &config.collect_prefix_label, &config.collect_prefix_label_raw, &config.collect_prefix_data };
    char str[256];
    for(int i=0;i<3;i++){
      if( !create_dir( dir[i]->c_str() ) ){
        return false;
      }
    }
  }

  ROS_INFO("object_type : %d", (int)config.object_type);
  ROS_INFO("save : %d", (int)config.save_mode);
  ROS_INFO("load : %d", (int)config.load_mode);
  ROS_INFO("action_server : %d", (int)config.action_server_mode);
  ROS_INFO("collect : %d", (int)config.collect_mode);

  ROS_INFO("show_result : %d", (int)config.show_result);
  ROS_INFO("save_result : %d", (int)config.save_result);
  ROS_INFO("save_file_prefix : %s", config.save_file_prefix.c_str());
  ROS_INFO("result_save_path : %s", config.result_save_path.c_str());
  ROS_INFO("data_save_path : %s", config.data_save_path.c_str());

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
  ROS_INFO("th_region_color : %d", config.th_region_color);
  
  ROS_INFO("min_label_pix_num : %d", config.min_label_pix_num);
  ROS_INFO("max_label_pix_num : %d", config.max_label_pix_num);
  ROS_INFO("min_label_ratio : %.3lf", config.min_label_ratio);
  ROS_INFO("max_label_ratio : %.3lf", config.max_label_ratio);
  ROS_INFO("dis_remove_edge : %.3lf", config.dis_remove_edge);
  ROS_INFO("min_label_size_x : %d", config.min_label_size_x);
  ROS_INFO("min_label_size_y : %d", config.min_label_size_y);
  ROS_INFO("label_mean_rgb : %d", config.label_mean_rgb);
  ROS_INFO("label_multiply_rgb : %.3lf", config.label_multiply_rgb);
  ROS_INFO("pc_dis2plane : %.3lf", config.pc_dis2plane);
  ROS_INFO("find_plane_radius : %d", config.find_plane_radius);

  ROS_INFO("box_size_x : %.3lf", config.box_size_x);
  ROS_INFO("box_size_y : %.3lf", config.box_size_y);
  ROS_INFO("box_size_z : %.3lf", config.box_size_z);
  ROS_INFO("box_label_shift_x : %.3lf", config.box_label_shift_x);
  ROS_INFO("basket_size_x : %.3lf", config.basket_size_x);
  ROS_INFO("basket_size_y : %.3lf", config.basket_size_y);
  ROS_INFO("basket_size_z : %.3lf", config.basket_size_z);
  ROS_INFO("label_width : %.3lf", config.label_width);
  ROS_INFO("label_height : %.3lf", config.label_height);
  ROS_INFO("label_size_error : %.3lf", config.label_size_error);

  
  {
    tf::Vector3 v = config.tf_cam.getOrigin();
    tf::Quaternion q = config.tf_cam.getRotation();
    ROS_INFO("tf_cam trans: %.3lf, %.3lf, %.3lf", v.x(), v.y(), v.z());
    ROS_INFO("tf_cam quat : %.3lf, %.3lf, %.3lf, %.3lf", q.x(), q.y(), q.z(), q.w());
  }
  
  if( config.result_save_path.size()>0 ){
    if( !create_dir( config.result_save_path.c_str() ) ){
      return false;
    }
  }
  if( config.data_save_path.size()>0 ){
    if( !create_dir( config.data_save_path.c_str() ) ){
      return false;
    }
  }
  config.save_file_prefix = config.data_save_path + config.save_file_prefix;
  return true;
}

void create_filename( const std::string &prefix, char *str
    , int cnt, int label){
  std::time_t t = std::time(0);   // get time now
  std::tm* now = std::localtime(&t);
  sprintf(str, "%s%02d%02d%02d-%02d%02d%02d-%03d-%d.bmp", prefix.c_str()
    , now->tm_year + 1900 - 2000
    , now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec
    , cnt, label );
}


#endif
