#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cobot_pick/common.h"
#include "cobot_pick/config_find_object.h"
#include "cobot_pick/cROSData.h"
#include "cobot_pick/cConvert3D.h"
#include "cobot_pick/cSegment.h"
#include "cobot_pick/cSelectObject.h"
#include <cobot_pick/CobotFindObjectAction.h>
#include "cobot_pick/cFindLabel.h"
#include <actionlib/server/simple_action_server.h>

/*sensor_msgs::CameraInfo msg_cam_info;
sensor_msgs::Image msg_depth, msg_col;
sensor_msgs::PointCloud2 msg_pc_org*/

sensor_msgs::PointCloud2 msg_pc_seg, msg_pc_plane;
cv_bridge::CvImagePtr p_img_col;
bool b_pub = false;
cSegment seg;
cSelectObject select_obj;
cFindLabel find_label;
ros::Publisher *p_pc_pub_pick = NULL;

void process_data()
{
  ROS_INFO("start process");
  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;

  p_img_col = cv_bridge::toCvCopy(ros_col.msg, sensor_msgs::image_encodings::BGR8);
  cConvert3D::convert_pc(ros_depth.msg, ros_cam_info.msg, ros_col.msg, cloud_rgb);
  pcl::toROSMsg(cloud_rgb, ros_pc.msg);

  find_label.run(p_img_col->image, cloud_rgb);
  pcl::toROSMsg(select_obj.plane_pc, msg_pc_plane);

  ros_pc.msg.header.frame_id = FRAME_CAMERA;
  msg_pc_seg.header.frame_id = FRAME_CAMERA;
  msg_pc_plane.header.frame_id = FRAME_CAMERA;
}

void execute_find_object(const cobot_pick::CobotFindObjectGoalConstPtr &goal, actionlib::SimpleActionServer<cobot_pick::CobotFindObjectAction> *as)
{
  cobot_pick::CobotFindObjectFeedback feedback;
  cobot_pick::CobotFindObjectResult result;
  cROSData::start_sub();
  ros::Rate loop_rate(20);
  bool b_ok = false;
  ros::Time t = ros::Time::now();
  if( p_pc_pub_pick )
    find_label.clear_markers();
  for (;;)
  {
    loop_rate.sleep();
    if (cROSData::is_all_saved())
    {
      feedback.percent_complete = 0.1;
      as->publishFeedback(feedback);
      process_data();
      /*      for(int i=0;i<select_obj.pick_poses.size();i++){
        result.poses.push_back(select_obj.pick_poses[i].pose);
        result.labels.push_back(select_obj.pick_poses[i].label);
      }*/

      result.poses.resize(find_label.planes.size());
      result.labels.resize(find_label.planes.size());
      printf("results :\n");
      for (int i = 0; i < find_label.planes.size(); i++)
      {
        cPlane &plane = find_label.planes[i];
        tPickPose tp;
        plane.get_pick_pose(tp);
        result.poses[plane.order] = tp.pose;
        result.labels[plane.order] = tp.label;
        printf("label[%d] : %s\n", plane.order, tp.label.c_str());
      }
      as->setSucceeded(result);
      printf("action end.\n");
      b_pub = true;
      return;
    }
    if (!ros::ok())
    {
      ROS_WARN("ROS was shutdowned while action is running");
      b_pub = true;
      return;
    }
    if ((ros::Time::now() - t).toSec() > 10.0)
    {
      ROS_ERROR("Action does not finished within 5 sec.");
      printf("saved : %d, %d, %d\n", (int)ros_cam_info.is_saved(), (int)ros_depth.is_saved(), (int)ros_col.is_saved());
      b_pub = true;
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
    unsigned int aa[128];
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);

    srand(spec.tv_nsec);
    //    srand((int)ros::Time::now().toNSec());
    for (int i = 0; i < 128; i++)
    {
      aa[i] = (unsigned int)rand();
    }
    InitMtEx(aa, 128);
  }
  
  if( !get_config() )
    return -1;

  if (config.save_mode)
  {
    cROSData::save_mode();
    ros::Rate r(10);

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
      b_pub = false;
      process_data();
      b_pub = true;
      for (int i = 0; i < find_label.planes.size(); i++)
      {
        cPlane &plane = find_label.planes[i];
        tPickPose tp;
        plane.get_pick_pose(tp);
        printf("label[%d] : %s\n", plane.order, tp.label.c_str());
      }
      //cROSData::load_mode(msg_cam_info, msg_depth, msg_col, cloud_rgb);
    }
    else if (config.action_server_mode)
    {
      action_server.start();
      printf("start action server\n");
    }

    ros::Publisher pc_pub_org = n.advertise<sensor_msgs::PointCloud2>("/my_pc_org", 20), pc_pub_seg = n.advertise<sensor_msgs::PointCloud2>("/my_pc_seg", 20), pc_pub_plane = n.advertise<sensor_msgs::PointCloud2>("/my_pc_plane", 20), pc_pub_plane_frames = n.advertise<visualization_msgs::Marker>("/my_pc_plane_frames", 20), pc_pub_normal = n.advertise<visualization_msgs::Marker>("/my_pc_normal", 20);
    ros::Publisher pc_pub_pick = n.advertise<visualization_msgs::Marker>("/my_pc_pick", 20);
    p_pc_pub_pick = &pc_pub_pick;
    ros::Time t_loop = ros::Time::now();
    ros::Rate r(1);
    while (ros::ok())
    {
      ros::spinOnce();
      if (b_pub && (ros::Time::now() - t_loop).toSec() > 1.0)
      {
        ros_pc.msg.header.stamp = msg_pc_seg.header.stamp = msg_pc_plane.header.stamp = t_loop = ros::Time::now();
        pc_pub_org.publish(ros_pc.msg);
        pc_pub_seg.publish(msg_pc_seg);
        pc_pub_plane.publish(msg_pc_plane);
        
        for (int i = find_label.pick_markers.size() - 1; i >= 0; i--){
          find_label.pick_markers[i].header.stamp = find_label.frame_markers[i].header.stamp = ros::Time::now();
          pc_pub_pick.publish(find_label.pick_markers[i]);
          pc_pub_pick.publish(find_label.frame_markers[i]);
        }
      }
      if (config.show_result)
      {
        char k = (char)cv::waitKey(1000);
        if (k == 27 || k == 'q')
        {
          break;
        }
      }
      else
      {
        r.sleep();
      }
    }
  }
  cROSData::stop_sub();
  ROS_INFO("stop");
  /*  while(!ros::isShuttingDown()){
    r.sleep();
  }*/
  return 0;
}
