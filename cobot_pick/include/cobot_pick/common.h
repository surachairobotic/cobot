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

#endif
