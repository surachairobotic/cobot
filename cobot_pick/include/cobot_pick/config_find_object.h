#ifndef __CONFIG_FIND_OBJECT_H__
#define __CONFIG_FIND_OBJECT_H__

#include <string>

struct tConfig{

  bool save_mode, load_mode, action_server_mode, show_result;
  std::string save_file_prefix, result_save_path;

  // segment
  double normal_th_ang, th_pointcloud_distance;
    //, norm_scale1, norm_scale2
    //, norm_th_curvature, segment_radius
    
  int normal_win_size, normal_thread_num;

  // select object
  int plane_reg_min, plane_reg_max, plane_ransac_repeat_time, text_binarize_win_size
    , text_ransac_repeat_time;
  double plane_reg_ratio_min, plane_reg_ratio_max, warp_meter2pixel
    , plane_ransac_th_error, text_binarize_subtract, text_ransac_th_error;


  tConfig():
    save_mode(false), load_mode(false), show_result(false), action_server_mode(false)
    , save_file_prefix("/home/tong/catkin_ws/src/cobot/cobot_pick/src/data/box")
    , result_save_path("/home/tong/catkin_ws/src/cobot/cobot_pick/results")

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
    {}
} config;

#endif