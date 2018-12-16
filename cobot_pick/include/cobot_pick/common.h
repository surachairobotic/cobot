#ifndef __COMMON_H__
#define __COMMON_H__

const double INVALID_POINT = 0.0;

#define POW2(x) ((x)*(x))
#define IS_VALID_POINT(p)  ((p).z!=INVALID_POINT)
#define SET_INVALID_POINT(p)  ((p).z = INVALID_POINT)
//#define DIS2(p,q) (POW2((p).x-(q).x)+POW2((p).y-(q).y)+POW2((p).z-(q).z))

const std::string save_path("/home/tong/catkin_ws/src/cobot/cobot_pick/");

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

  // segment
  double norm_scale1, norm_scale2, norm_th_rad, threshold_pointcloud_distance;
    //, norm_th_curvature, segment_radius
    
  int min_cluster_size, max_cluster_size, norm_k_search1, norm_k_search2
    , norm_k_search_my, norm_thread;

  // select object
  int reg_min_pix, reg_max_pix, ransac_repeat_time, th_text_binary;
  double reg_min_ratio, reg_max_ratio, ransac_th_error;
  double warp_meter2pixel;

  tConfig():
    // segment
      norm_scale1(0.007), norm_scale2(0.015)
    , norm_k_search1(0), norm_k_search2(0)
    , norm_th_rad(0.1), threshold_pointcloud_distance(0.01)
//    , norm_th_curvature(0.9),segment_radius(0.01), min_cluster_size(10), max_cluster_size(50000)
    , norm_k_search_my(4), norm_thread(1)
    

    // select object
    ,reg_min_pix(0),reg_max_pix(99999999)
    ,reg_min_ratio(1.0), reg_max_ratio(1000000.0), ransac_repeat_time(100)
    , ransac_th_error(0.01), warp_meter2pixel(2000.0)
    , th_text_binary(120)
    {}
} config;

#endif
