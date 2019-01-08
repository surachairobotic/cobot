#ifndef __CONFIG_CALIBRATE_H__
#define __CONFIG_CALIBRATE_H__

#include <string>

struct tConfig{

  bool load_mode, show_result, save_mode, action_server_mode;
  std::string save_file_prefix;

  int chess_num[2], plane_ransac_repeat_time, bruteforce_num;
  double chess_size[2], chess_pos[4][3], plane_ransac_th_error, bruteforce_range;

  tConfig():
    load_mode(false), show_result(false), save_mode(false), action_server_mode(false)
    , save_file_prefix("/home/tong/catkin_ws/src/cobot/cobot_pick/src/data/calibrate")
    , plane_ransac_repeat_time(1000), plane_ransac_th_error(0.002)
    , chess_num{0}, chess_size{0.0}
    , chess_pos{{0.0}}
    , bruteforce_range(0.1), bruteforce_num(100)
    {}
} config;

#endif