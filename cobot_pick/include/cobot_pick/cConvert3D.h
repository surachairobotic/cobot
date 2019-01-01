
#ifndef __CONVERT_PC_H__
#define __CONVERT_PC_H__

// https://github.com/ros-perception/image_pipeline/blob/indigo/depth_image_proc/include/depth_image_proc/depth_conversions.h

#include "cobot_pick/common.h"
#include <image_geometry/pinhole_camera_model.h>


 // Encapsulate differences between processing float and uint16_t depths
 template<typename T> struct DepthTraits {};
 
 template<>
 struct DepthTraits<uint16_t>
 {
   static inline bool valid(uint16_t depth) { return depth != 0; }
   static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
   static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
   static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
 };
 
template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
  static inline float fromMeters(float depth) { return depth; }

  static inline void initializeBuffer(std::vector<uint8_t>& buffer)
  {
    float* start = reinterpret_cast<float*>(&buffer[0]);
    float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};


class cConvert3D{
private:
  static float center_x, center_y, constant_x, constant_y;

public:

  static void get_vector(const int u, const int v, pcl::PointXYZ &p){
    const float depth = 100;
    p.x = (u - center_x) * depth * constant_x;
    p.y = (v - center_y) * depth * constant_y;
    p.z = DepthTraits<uint16_t>::toMeters(depth);
  }

  static void convert_pc(const sensor_msgs::Image& img_depth
      , const sensor_msgs::CameraInfo& cam_info
      , const sensor_msgs::Image& img_col
      , pcl::PointCloud<pcl::PointXYZRGB> &cloud){
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(cam_info);

    ROS_INFO("depth : w=%d, h=%d, step=%d, len=%d"
      , (int)img_depth.width, (int)img_depth.height
      , (int)img_depth.step, (int)img_depth.data.size());

    center_x = model.cx();
    center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = DepthTraits<uint16_t>::toMeters( 1 );
    constant_x = unit_scaling / model.fx();
    constant_y = unit_scaling / model.fy();
  //  uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&img_depth.data[0]);
    int row_step = img_depth.width;

    cloud.width = img_depth.width;
    cloud.height = img_depth.height;
    cloud.resize(cloud.width*cloud.height);
    cloud.is_dense = false;

    const uint16_t *p_depth = reinterpret_cast<const uint16_t*>(&img_depth.data[0]);
    const uint8_t *p_col = reinterpret_cast<const uint8_t*>(&img_col.data[0]);
    int cnt = 0;
    for (int v = 0; v < (int)img_depth.height; ++v){
      const int i2 = v*row_step;
      for (int u = 0; u < (int)img_depth.width; ++u){
        const int i = i2 + u;
        const uint16_t depth = p_depth[i];
        const uint8_t *c = p_col+i*3;
        pcl::PointXYZRGB &p = cloud.points[i];
        if( depth==0 ){
          p.x = p.y = p.z = INVALID_POINT;
        }
        else{
          p.x = (u - center_x) * depth * constant_x;
          p.y = (v - center_y) * depth * constant_y;
          p.z = DepthTraits<uint16_t>::toMeters(depth);
          p.r = c[0];
          p.g = c[1];
          p.b = c[2];
          cnt++;
        }
      }
    }
    ROS_INFO("valid point : %d , %.2lf%%", cnt, double(cnt)/cloud.points.size());
  //  convert<uint16_t>(depth, cloud_msg, model_);
  }
};

float cConvert3D::center_x, cConvert3D::center_y, cConvert3D::constant_x, cConvert3D::constant_y;

#endif