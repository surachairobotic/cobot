
#ifndef __CROSDATA_H__
#define __CROSDATA_H__

#include "ros/ros.h"
#include <image_geometry/pinhole_camera_model.h>
#include <assert.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class cROSData{
protected:
  bool b_saved;
  ros::Subscriber sub;

  // write
  template<typename T>
  static void write_vector(FILE *fp, const T &vec){
    if( vec.size()==0 )
      return;
    int size = sizeof(vec[0]), len = vec.size();
    fwrite( &len, sizeof(int), 1, fp);
    for(int i=0;i<vec.size();i++)
      fwrite( &vec[i], size, 1, fp);
  }

  template<typename T>
  static void write(FILE *fp, const T &data){
    fwrite( &data, sizeof(T), 1, fp);
  }

  // read
    template<typename T>
  static void read_array(FILE *fp, T &vec, int len){
    int len2;
    fread( &len2, sizeof(int), 1, fp);
    assert(len==len2);
    read_multiple(fp, vec);
  }

  template<typename T>
  static void read_vector(FILE *fp, T &vec){
    int len;
    fread( &len, sizeof(int), 1, fp);
    vec.resize(len);
    read_multiple(fp, vec);
  }

  template<typename T>
  static void read_multiple(FILE *fp, T &vec){
    int len = vec.size(), size = sizeof(vec[0]);
    char *buf = new char[len*size];
    fread( buf, size, len, fp );
    for(int i=len-1;i>=0;i--)
      memcpy( &vec[i], buf + i*size, size );
    delete [] buf;
  }

  template<typename T>
  static void read(FILE *fp, T &data){
    fread( &data, sizeof(T), 1, fp);
  }

  static bool check_end(FILE *fp){
    long int pos = ftell(fp), end;
    fseek(fp,  0L, SEEK_END);;
    end = ftell(fp);
    if (pos!= end){
      ROS_ERROR("Invalid file read size : pos = %ld / %ld", pos, end);
      return false;
    }
    return true;
  }


public:
  static std::string file_prefix;
  static bool b_save_mode, b_load_mode;
  

  cROSData():b_saved(false){}

  bool is_saved(){ return b_saved; }

  static void save_mode(ros::NodeHandle &n);
  static void load_mode(sensor_msgs::CameraInfo& msg_cam_info
    , sensor_msgs::Image& msg_depth
    , pcl::PointCloud<pcl::PointXYZRGB> &cloud );
  static bool is_all_saved();
  static void stop();
};

class cROSCameraInfo : public cROSData{
private:
  const std::string get_file_name(){ return (file_prefix+"_cam_info.bin"); }
  
public:

  bool save(const sensor_msgs::CameraInfo& msg){
    FILE *fp;
    fp = fopen( get_file_name().c_str(), "wb" );
    if(fp){
      write(fp, msg.height);
      write(fp, msg.width);
      write_vector(fp, msg.distortion_model);
      write_vector(fp, msg.D);
      write_vector(fp, msg.K);
      write_vector(fp, msg.R);
      write_vector(fp, msg.P);
      write(fp, msg.binning_x);
      write(fp, msg.binning_y);
      write(fp, msg.roi);
      fclose(fp);
      b_saved = true;
      ROS_INFO("cam info saved");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  bool load(sensor_msgs::CameraInfo& msg){
    FILE *fp;
    fp = fopen( get_file_name().c_str(), "rb" );
    if(fp){
      read(fp, msg.height);
      read(fp, msg.width);
      read_vector(fp, msg.distortion_model);
      read_vector(fp, msg.D);
      read_array(fp, msg.K, 9);
      read_array(fp, msg.R, 9);
      read_array(fp, msg.P, 12);
      read(fp, msg.binning_x);
      read(fp, msg.binning_y);
      read(fp, msg.roi);
      check_end(fp);
      fclose(fp);
      ROS_INFO("cam info loaded");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  void cb(const sensor_msgs::CameraInfo &msg){
    if( b_save_mode && !b_saved )
      save(msg);
  }
};



class cROSDepth : public cROSData{
private:
  const std::string get_file_name(){ return (file_prefix+"_depth.bin"); }
  
public:
  bool save(const sensor_msgs::Image& msg){
    FILE *fp;
    fp = fopen( get_file_name().c_str(), "wb" );
    if(fp){
      write(fp, msg.height);
      write(fp, msg.width);
      write_vector(fp, msg.encoding);
      write(fp, msg.is_bigendian);
      write(fp, msg.step);
      write_vector(fp, msg.data);
      fclose(fp);
      b_saved = true;
      ROS_INFO("depth saved");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  bool load(sensor_msgs::Image& msg){
    FILE *fp;
    fp = fopen( get_file_name().c_str(), "rb" );
    if(fp){
      read(fp, msg.height);
      read(fp, msg.width);
      read_vector(fp, msg.encoding);
      read(fp, msg.is_bigendian);
      read(fp, msg.step);
      read_vector(fp, msg.data);
      check_end(fp);
      fclose(fp);
      ROS_INFO("depth loaded");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  void cb(const sensor_msgs::Image &msg){
    if( b_save_mode && !b_saved )
      save(msg);
  }

};



class cROSPointCloud : public cROSData{
private:
  const std::string get_file_name(){ return (file_prefix+"_pc.pcd"); }
  
public:
  bool save(const sensor_msgs::PointCloud2& msg){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg( msg, cloud );
    ROS_INFO("pc : %s", get_file_name().c_str());
    pcl::io::savePCDFileASCII (get_file_name().c_str(), cloud);
    b_saved = true;
    ROS_INFO("pc saved");
    return true;
  }

  bool load(pcl::PointCloud<pcl::PointXYZRGB> &cloud){
    pcl::io::loadPCDFile(get_file_name().c_str(), cloud);
    ROS_INFO("pc loaded");
    return true;
  }

  void cb(const sensor_msgs::PointCloud2 &msg){
    if( b_save_mode && !b_saved )
      save(msg);
  }
};

std::string cROSData::file_prefix;
bool cROSData::b_save_mode = false, cROSData::b_load_mode = false;

cROSCameraInfo ros_cam_info;
cROSDepth ros_depth;
cROSPointCloud ros_pc;

void cb_cam_info(const sensor_msgs::CameraInfo& msg){ ros_cam_info.cb(msg); }
void cb_depth(const sensor_msgs::Image& msg){ ros_depth.cb(msg); }
void cb_pc(const sensor_msgs::PointCloud2& msg){ ros_pc.cb(msg); }


void cROSData::save_mode(ros::NodeHandle &n){
  ros_cam_info.sub = n.subscribe("/camera/aligned_depth_to_color/camera_info", 10, cb_cam_info);
  ros_depth.sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 10, cb_depth);
  ros_pc.sub = n.subscribe("/camera/depth/color/points", 10, cb_pc);
  ROS_INFO("save mode ...");
  b_save_mode = true;
}


void cROSData::load_mode(sensor_msgs::CameraInfo& msg_cam_info
    , sensor_msgs::Image& msg_depth
    , pcl::PointCloud<pcl::PointXYZRGB> &cloud ){
  ROS_INFO("load mode ...");
  ros_cam_info.load( msg_cam_info );
  ros_depth.load( msg_depth );
  ros_pc.load( cloud );
  ROS_INFO("load OK");
  b_load_mode = true;
}

bool cROSData::is_all_saved(){
  return ros_cam_info.is_saved() && ros_depth.is_saved() && ros_pc.is_saved();
}

void cROSData::stop(){
  ros_pc.sub.shutdown();
  ros_cam_info.sub.shutdown();
  ros_depth.sub.shutdown();
}
#endif