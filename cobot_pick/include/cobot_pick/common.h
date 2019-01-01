#ifndef __COMMON_H__
#define __COMMON_H__

const double INVALID_POINT = 0.0;

#define POW2(x) ((x)*(x))
#define IS_VALID_POINT(p)  ((p).z!=INVALID_POINT)
#define SET_INVALID_POINT(p)  ((p).z = INVALID_POINT)
//#define DIS2(p,q) (POW2((p).x-(q).x)+POW2((p).y-(q).y)+POW2((p).z-(q).z))

//const std::string config.result_save_path("/home/tong/catkin_ws/src/cobot/cobot_pick/");
const int cols[6][3] = {
  {255, 50, 50},
  {50, 255, 50},
  {50, 50, 255},
  {255, 250, 50},
  {50, 255, 250},
  {250, 50, 255}};

template<typename T>
inline double DIS2(const T &p1, const T &p2){
  return POW2(p1.x-p2.x)+POW2(p1.y-p2.y)+POW2(p1.z-p2.z);
}
template<typename T>
inline double LEN2(const T &p){
  return POW2(p.x)+POW2(p.y)+POW2(p.z);
}

template<typename T, typename T2>
inline void CROSS_3D(T &p_ans, const T &p1, const T2 &p2){
  p_ans.x = p1.y*p2.z - p1.z*p2.y;
  p_ans.y = p1.z*p2.x - p1.x*p2.z;
  p_ans.z = p1.x*p2.y - p1.y*p2.x;
}

template<typename T,typename T2>
inline double INNER_3D(const T &p1, const T2 &p2){
  return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
}

template<typename T,typename T2,typename T3>
inline void MINUS_3D(T &p_ans, const T2 &p1, const T3 &p2){
  p_ans.x = p1.x - p2.x;
  p_ans.y = p1.y - p2.y;
  p_ans.z = p1.z - p2.z;
}

template<typename T>
inline void NORMALIZE_3D(T &p){
  double len = 1.0/sqrt(LEN2(p));
  p.x*= len;
  p.y*= len;
  p.z*= len;
}

template<typename T>
inline void NORMALIZE_3D(T &p_ans, const T &p){
  double len = 1.0/sqrt(LEN2(p));
  p_ans.x = p.x*len;
  p_ans.y = p.y*len;
  p_ans.z = p.z*len;
}


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
