
#ifndef __CROSDATA_H__
#define __CROSDATA_H__

#include "cobot_pick/common.h"
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
  std::string name_ext;
  static ros::NodeHandle *p_nh;

  const std::string get_file_name(){ return (config.save_file_prefix+name_ext); }

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
  sensor_msgs::CameraInfo msg_cam_info;
  sensor_msgs::Image msg_depth, msg_col;
  sensor_msgs::PointCloud2 msg_pc_org;
  

  cROSData():b_saved(false){}
  cROSData(const std::string &_name):b_saved(false), name_ext(_name){}

  inline bool is_saved(){ return b_saved; }
  inline void reset_saved(){ b_saved = false; }

  static inline void set_nh(ros::NodeHandle &nh){ p_nh = &nh; }
  static void save_mode();
  static void load_mode();
  static bool is_all_saved();
  static void start_sub();
  static void stop_sub();
};

class cROSCameraInfo : public cROSData{
public:
  sensor_msgs::CameraInfo msg;


  cROSCameraInfo():cROSData("_cam_info.bin"){}

  bool save(const sensor_msgs::CameraInfo& msg, const char *fname=NULL){
    FILE *fp;
    fp = fopen( fname ? fname : get_file_name().c_str(), "wb" );
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
      ROS_INFO("cam info saved");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  bool load(){
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
    sub.shutdown();
    if( !b_saved ){
      if( config.save_mode ){
          save(msg);
      }
      if( config.action_server_mode ){
        this->msg = msg;
      }
      b_saved = true;
    }
  }
};



class cROSImage : public cROSData{
public:
  sensor_msgs::Image msg;


  cROSImage():cROSData("_img.bin"){}
  cROSImage(const std::string &_name):cROSData(_name){}

  bool save(const sensor_msgs::Image& msg, const char *fname=NULL){
    FILE *fp;
    fp = fopen( fname ? fname : get_file_name().c_str(), "wb" );
    if(fp){
      write(fp, msg.height);
      write(fp, msg.width);
      write_vector(fp, msg.encoding);
      write(fp, msg.is_bigendian);
      write(fp, msg.step);
      write_vector(fp, msg.data);
      fclose(fp);
      ROS_INFO("depth saved");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  bool load(){
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
      ROS_INFO("image loaded");
      return true;
    }
    else{
      ROS_ERROR("Cannot open file %s", get_file_name().c_str());
      return false;
    }
  }

  void cb(const sensor_msgs::Image &msg){
    sub.shutdown();
    if( !b_saved ){
      if( config.save_mode ){
          save(msg);
      }
      if( config.action_server_mode ){
        this->msg = msg;
      }
      b_saved = true;
    }
  }

};



class cROSPointCloud : public cROSData{
public:
  sensor_msgs::PointCloud2 msg;


  cROSPointCloud():cROSData("_pc.pcd"){}

  bool save(const sensor_msgs::PointCloud2& msg, const char *fname=NULL){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg( msg, cloud );
    pcl::io::savePCDFileASCII (fname ? fname : get_file_name().c_str(), cloud);
    ROS_INFO("pc saved");
    return true;
  }

  bool load(){
    pcl::io::loadPCDFile(get_file_name().c_str(), msg);
    ROS_INFO("pc loaded");
    return true;
  }

  void cb(const sensor_msgs::PointCloud2 &msg){
    sub.shutdown();
    if( !b_saved ){
      if( config.save_mode ){
          save(msg);
      }
      if( config.action_server_mode ){
        this->msg = msg;
      }
      b_saved = true;
    }
  }
};

cROSCameraInfo ros_cam_info;
cROSImage ros_depth("_depth.bin"), ros_col("_col.bin");
cROSPointCloud ros_pc;
ros::NodeHandle *cROSData::p_nh = NULL;

void cb_cam_info(const sensor_msgs::CameraInfo& msg){ ros_cam_info.cb(msg); }
void cb_depth(const sensor_msgs::Image& msg){ ros_depth.cb(msg); }
void cb_col(const sensor_msgs::Image& msg){ ros_col.cb(msg); }
void cb_pc(const sensor_msgs::PointCloud2& msg){ ros_pc.cb(msg); }


void cROSData::start_sub(){
  ros_cam_info.reset_saved();
  ros_depth.reset_saved();
  ros_col.reset_saved();
  ros_pc.reset_saved();

  std::string tf_prefix = config.object_type==OBJECT_BOX ? "/camera" : "/camera_basket";
  ros_cam_info.sub = p_nh->subscribe(tf_prefix+"/aligned_depth_to_color/camera_info", 10, cb_cam_info);
  ros_depth.sub = p_nh->subscribe(tf_prefix+"/aligned_depth_to_color/image_raw", 10, cb_depth);
  ros_col.sub = p_nh->subscribe(tf_prefix+"/color/image_raw", 10, cb_col);
//  ros_pc.sub = p_nh->subscribe("/camera/depth/color/points", 10, cb_pc);

  ROS_INFO("start sub");
}

void cROSData::stop_sub(){
  ros_pc.sub.shutdown();
  ros_cam_info.sub.shutdown();
  ros_col.sub.shutdown();
  ros_depth.sub.shutdown();
}

void cROSData::save_mode(){
  ROS_INFO("save mode ...");
  start_sub();
}


/*
void cROSData::load_mode(sensor_msgs::CameraInfo& msg_cam_info
    , sensor_msgs::Image& msg_depth
    , sensor_msgs::Image& msg_col
    , pcl::PointCloud<pcl::PointXYZRGB> &cloud ){
  ROS_INFO("load mode ...");
  ros_cam_info.load( msg_cam_info );
  ros_depth.load( msg_depth );
  ros_col.load( msg_col );
//  ros_pc.load( cloud );
  ROS_INFO("load OK");
}  */
void cROSData::load_mode(){
  ROS_INFO("load mode ...");
  ros_cam_info.load();
  ros_depth.load();
  ros_col.load();
//  ros_pc.load();
  ROS_INFO("load OK");
}

bool cROSData::is_all_saved(){
  return ros_cam_info.is_saved() && ros_depth.is_saved() 
    && ros_col.is_saved(); //&& ros_pc.is_saved() ;
}

/*
template<typename PointT>
void reorganize(pcl::PointCloud<PointT> &cloud){
  const int shift = 213, width = 640;
  const float e = 0.00001;
  int cnt = 0;
  double min_z = 9999999, max_z = -9999999;
  std::vector<PointT, Eigen::aligned_allocator<PointT> > ps(cloud.points);
  cloud.width = width;
  cloud.height = cloud.points.size() / width;
  ROS_INFO("point size : %d / %d, same = %d", (int)cloud.points.size()
    , (int)(cloud.width * cloud.height)
    , int(cloud.points.size()==(cloud.width * cloud.height)));
  for(int i=cloud.height-1;i>=0;i--){
    const int i2 = i*cloud.width;
    for(int j=cloud.width-1;j>=0;j--){
      const PointT &p1 = ps[i2+j];
      PointT &p2 = cloud.points[i2+(j+width-shift)%shift];
      if(fabs(p1.x)<e && fabs(p1.y)<e && fabs(p1.z)<e ){
        p2.x = p2.y = p2.z = std::numeric_limits<float>::quiet_NaN(); 
      }
      else{
        p2 = p1;
        if( p1.z>max_z)
          max_z = p1.z;
        if( p1.z<min_z)
          min_z = p1.z;
        cnt++;
      }
    }
  }
  ROS_INFO("cnt valid : %d", cnt);
  ROS_INFO("min max z : %.3lf, %.3lf", min_z, max_z);
  cloud.is_dense = false;
}*/


#endif
