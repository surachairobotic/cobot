
#ifndef __CSELECTOBJECT_H__
#define __CSELECTOBJECT_H__

#include "cobot_pick/common.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cobot_pick/cSegment.h"
#include "cobot_pick/zdsfmt.h"
#include "cobot_pick/cLabeling.h"
#include "cobot_pick/cPCA_2D.h"
#include "cobot_pick/cOCR.h"
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>


struct tPickPose{
  geometry_msgs::Pose pose;
  std::string label;
};

struct tPoint2i
{
  int i, j, u, v;
  const pcl::PointXYZRGB *p;
  tPoint2i() {}
  tPoint2i(int _i, int _j, pcl::PointXYZRGB *_p) : i(_i), j(_j), p(_p) {}
};

void fill_region(const tRegInfo *p_reg, const cv::Mat &img_label, cv::Mat &img){
  img = cv::Mat( p_reg->y2-p_reg->y1+1
      , p_reg->x2-p_reg->x1+1, CV_8UC1);
  for (int i = img.rows - 1; i >= 0; i--)
  {
    unsigned short *p_label = (unsigned short *)(img_label.data + img_label.step * (i + p_reg->y1)) + p_reg->x1;
    unsigned char *p = (unsigned char *)(img.data + img.step * i);
//      unsigned char *p = (unsigned char *)(img.data + img.step * i);
    for (int j = img.cols - 1; j >= 0; j--)
    {
      p[j] = (p_label[j] == p_reg->n_label) ? 0 : 255;
    }
  }

  // fill
  {
    cLabeling label;
    cv::Mat img_label;
    label.ExecBin(img, img_label);
    for(int k=label.reg_info.size()-1;k>=0;k--){
      const tRegInfo &r = label.reg_info[k];
      if( r.x1==0 || r.x2==img.cols-1 || 
        r.y1==0 || r.y2==img.rows-1 ){
        continue;
      }
      for(int i=r.y1;i<=r.y2;i++){
        unsigned short *p_label = (unsigned short *)(img_label.data + img_label.step * i);
        unsigned char *p = (unsigned char *)(img.data + img.step * i);
        for(int j=r.x1;j<=r.x2;j++){
          if( p_label[j]==r.n_label ){
            p[j] = 0;
          }
        }
      }
    }
    cv::bitwise_not(img, img);
  }
}

void binarize(const cv::Mat &src, cv::Mat &dst, const int block_size
    , const int mean_bias){
  int *sum_row = new int[block_size*2+1];
  int x1, x2, y1, y2, n;
  cv::Mat img_gray = cv::Mat( src.size(), CV_8UC1 );

  dst = cv::Mat( src.size(), CV_8UC1);
  cv::cvtColor( src, img_gray, cv::COLOR_BGR2GRAY);
  const unsigned char *p = img_gray.data;
  unsigned char *p2 = dst.data;
  const int w = src.cols, h = src.rows;
  for(int i=0;i<h;i++){
    
    y1 = i-block_size;
    y2 = i+block_size;
    if( y1<0 ) y1 = 0;
    if( y2>=h ) y2 = h-1;
    for(int j=0;j<w;j++){
      
      x1 = j-block_size;
      x2 = j+block_size;
      if( x1<0 ) x1 = 0;
      if( x2>=w ) x2 = w-1;

      if( j==0 ){
        n = 0;
        memset( sum_row, 0, sizeof(int)*(block_size*2+1) );
        for(int y=y1;y<=y2;y++){
          for(int x=x1;x<=x2;x++){
            int v = p[w*y + x];
            if( v>0 ){
              n++;
              //assert( y-y1>=0 && y-y1<block_size*2+1 );
              sum_row[y-y1]+= v;
            }
          }
        }
      }
      else{
        if( j-block_size>0 ){
          int x0 = x1-1;
          for(int y=y1;y<=y2;y++){
            int v =  p[w*y+x0];
            if( v>0 ){
              n--;
              //assert( y-y1>=0 && y-y1<block_size*2+1 );
              sum_row[y-y1]-= v;
            }
          }
        }
        if( j+block_size<w ){
          for(int y=y1;y<=y2;y++){
            int v = p[w*y+x2];
            if( v>0 ){
              n++;
              //assert( y-y1>=0 && y-y1<block_size*2+1 );
              sum_row[y-y1]+= v;
            }
          }
        }
      }

      int sum = 0;
      for(int y=y1;y<=y2;y++){
        //assert( y-y1>=0 && y-y1<block_size*2+1 );
        sum+= sum_row[y-y1];
      }

      /*{
        int s = 0, n2 = 0;
        for(int y=y1;y<=y2;y++){
          for(int x=x1;x<=x2;x++){
            int v = p[w*y+x];
            if( v>0 ){
              n2++;
              s+= v;
            }
          }
        }
        assert( s==sum && n==n2 );
      }*/
      p2[w*i+j] = ( n>0 && p[w*i+j] + mean_bias >= sum/n ) ? 255 : 0;
    }
  }
  delete [] sum_row;
}


class cPlane
{
private:
  template <typename T>
  void get_uv(const pcl::PointXYZ &p, T &u, T &v)
  {
    pcl::PointXYZ p0;
    MINUS_3D(p0, p, point);
    {
      pcl::PointXYZ pc;
      assert(fabs(INNER_3D(normal, p0)) < 0.00001);
    }
    u = INNER_3D(x_axis, p0);
    v = INNER_3D(y_axis, p0);
  }

public:
  pcl::PointXYZ point, normal, x_axis, y_axis, center, frames[4];
  const tRegInfo *p_reg;
  //  double corners_2d[4][2];
  pcl::PointXYZ corners[4];
  cv::Mat img_plane;

  cPlane():p_reg(NULL) {}
  cPlane(const tRegInfo *r):p_reg(r) {}

  void set_plane(const pcl::PointXYZRGB &_p, const pcl::PointXYZ &_normal, const pcl::PointXYZ &_x_axis)
  {
    point.x = _p.x;
    point.y = _p.y;
    point.z = _p.z;
    normal = _normal;
    NORMALIZE_3D(x_axis, _x_axis);
    CROSS_3D(y_axis, normal, x_axis);
    NORMALIZE_3D(y_axis);
    assert( fabs(INNER_3D(normal, x_axis)) < 0.00001
      && fabs(INNER_3D(y_axis, x_axis)) < 0.00001 );
  }

  void warp(const cv::Mat &img_col, const cSegment *p_seg)
  {
    // create image
    const cv::Mat &img_label = p_seg->img_label;
    cv::Mat img;

    {
      cv::Mat img_filter;
      fill_region( p_reg, img_label, img_filter);
      img = cv::Mat( img_filter.size(), CV_8UC3, cv::Scalar(0));
      cv::Mat(img_col, cv::Rect(p_reg->x1, p_reg->y1, p_reg->x2 - p_reg->x1 + 1
        , p_reg->y2 - p_reg->y1 + 1))
        .copyTo(img, img_filter);
    }

    const double np = INNER_3D(normal, point);
    double min_u, max_u, min_v, max_v;
    min_u = min_v = 999999.0;
    max_u = max_v = -min_v;

    cv::Point2f warp_src[4], warp_dst[4];
    warp_src[0].x = 0;
    warp_src[0].y = 0;
    warp_src[1].x = img.cols - 1;
    warp_src[1].y = 0;
    warp_src[2].x = img.cols - 1;
    warp_src[2].y = img.rows - 1;
    warp_src[3].x = 0;
    warp_src[3].y = img.rows - 1;
    /*    warp_src[0].x = p_reg->x1;
    warp_src[0].y = p_reg->y1;
    warp_src[1].x = p_reg->x2;
    warp_src[1].y = p_reg->y1;
    warp_src[2].x = p_reg->x2;
    warp_src[2].y = p_reg->y2;
    warp_src[3].x = p_reg->x1;
    warp_src[3].y = p_reg->y2;*/
    //printf("warp src 1: %d, %d\n", p_reg->x1, p_reg->y1);
    //printf("warp src 2: %d, %d\n", p_reg->x2, p_reg->y2);
    for (int i = 3; i >= 0; i--)
    {
      cConvert3D::get_vector(warp_src[i].x + p_reg->x1, warp_src[i].y + p_reg->y1, corners[i]);
      // find corner 3d points on the plane
      pcl::PointXYZ &p = corners[i];
      const double k = np / INNER_3D(normal, p);
      p.x *= k;
      p.y *= k;
      p.z *= k;

      // check if the point is really on the plane
      {
        pcl::PointXYZ p2;
        MINUS_3D(p2, p, point);
        assert(fabs(INNER_3D(normal, p2)) < 0.000001);
      }

      get_uv(p, warp_dst[i].x, warp_dst[i].y);
      if (warp_dst[i].x < min_u)
        min_u = warp_dst[i].x;
      if (warp_dst[i].x > max_u)
        max_u = warp_dst[i].x;
      if (warp_dst[i].y < min_v)
        min_v = warp_dst[i].y;
      if (warp_dst[i].y > max_v)
        max_v = warp_dst[i].y;
    }
    for (int i = 3; i >= 0; i--)
    {
      warp_dst[i].x = (warp_dst[i].x - min_u) * config.warp_meter2pixel;
      warp_dst[i].y = (warp_dst[i].y - min_v) * config.warp_meter2pixel;
    }
    img_plane = cv::Mat(config.warp_meter2pixel * (max_v - min_v), config.warp_meter2pixel * (max_u - min_u), CV_8UC3);
    cv::Mat trans = cv::getPerspectiveTransform(warp_src, warp_dst);
    cv::warpPerspective(img, img_plane, trans, img_plane.size());
  }

  void find_center(const pcl::PointCloud<pcl::PointXYZRGB> &cloud
    , const cv::Mat &img_label){

    double min_x, min_y, max_x, max_y;
    min_x = min_y = 9999999.0;
    max_x = max_y = -min_x;
    for(int i=p_reg->y1;i<=p_reg->y2;i++){
      const unsigned short *pl = (unsigned short*)(img_label.data + img_label.step*i);
      const pcl::PointXYZRGB *pc = &cloud.points[i*img_label.cols];
      for(int j=p_reg->x1;j<=p_reg->x2;j++){
        if( pl[j]==p_reg->n_label && IS_VALID_POINT(pc[j]) ){
          pcl::PointXYZ p1, p2;
          // project pc to the plane
          MINUS_3D(p1, pc[j], point);
          double d = INNER_3D(normal, p1);
          p1.x-=normal.x*d;
          p1.y-=normal.y*d;
          p1.z-=normal.z*d;

          // find x,y
          double new_x = INNER_3D( p1, x_axis ), new_y = INNER_3D(p1, y_axis);
          if( new_x>max_x ) max_x = new_x;
          if( new_x<min_x ) min_x = new_x;
          if( new_y>max_y ) max_y = new_y;
          if( new_y<min_y ) min_y = new_y;

        }
      }
    }
    assert(min_x < 999999.0);
    double cx = 0.5*(min_x+max_x), cy = 0.5*(min_y+max_y);
    center.x = point.x + x_axis.x*cx + y_axis.x*cy;
    center.y = point.y + x_axis.y*cx + y_axis.y*cy;
    center.z = point.z + x_axis.z*cx + y_axis.z*cy;

    const double xy[4][2] = {{ max_x, max_y }
      , { max_x, min_y }
      , { min_x, min_y }
      , { min_x, max_y }};
    
    for(int i=0;i<4;i++){
      frames[i].x = point.x + x_axis.x*xy[i][0] + y_axis.x*xy[i][1];
      frames[i].y = point.y + x_axis.y*xy[i][0] + y_axis.y*xy[i][1];
      frames[i].z = point.z + x_axis.z*xy[i][0] + y_axis.z*xy[i][1];
    }
  }
};

class cSelectObject
{
private:
  std::vector<int> reg_index;
  cOCR ocr;

  void filter_region()
  {
    reg_index.clear();
    for (int i = p_seg->label.reg_info.size() - 1; i >= 0; i--)
    {
      const tRegInfo &r = p_seg->label.reg_info[i];
      if (r.pix_num < config.plane_reg_min || r.pix_num > config.plane_reg_max)
        continue;

      const int dx = r.x2 - r.x1 + 1, dy = r.y2 - r.y1 + 1;
      double ratio = dx > dy ? dx / double(dy) : dy / double(dx);
      if (ratio < config.plane_reg_ratio_min || ratio > config.plane_reg_ratio_max)
        continue;
      reg_index.push_back(i);
    }
    ROS_INFO("filter reg : %d / %d", (int)reg_index.size(), (int)p_seg->label.reg_info.size());
  }

  void find_planes()
  {

    // valid points for ransac
    std::vector<tPoint2i> vec_pix;
    const int w = p_seg->cloud->width, h = p_seg->cloud->height;
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &pnts = p_seg->cloud->points;

    // min distance between points that are picked up for ransac
    const double MIN_DIS_BETWEEN_POINTS2 = POW2(0.01);

    tPoint2i p2i;
    vec_pix.reserve(w * h * 0.2);
    for (int k = reg_index.size() - 1; k >= 0; k--)
    {
      tRegInfo &r = p_seg->label.reg_info[reg_index[k]];
      vec_pix.clear();

      // collect valid points for ransac
      for (int i = r.y1; i <= r.y2; i++)
      {
        const int i2 = i * w;
        for (int j = r.x1; j <= r.x2; j++)
        {
          const int j2 = i2 + j;
          if (!IS_VALID_POINT(pnts[j2]))
            continue;
          p2i.i = i;
          p2i.j = j;
          p2i.p = &pnts[j2];
          vec_pix.push_back(p2i);
        }
      }

      // ransac
      //double best_cen[3], best_normal[3];
      //pcl::PointXYZ best_cen, best_normal[2];
      int best_cnt = 0;
      cPlane best_plane(&r);
      for (int k1 = config.plane_ransac_repeat_time - 1; k1 >= 0; k1--)
      {
        const tPoint2i *ps[3];
        // randomize 3 points to create plane
        {
          int cnt2 = 5;
          bool b_ok2;
          do
          {
            b_ok2 = true;
            ps[0] = &vec_pix[NextMt() % vec_pix.size()];
            for (int i = 1; i < 3; i++)
            {
              bool b_ok;
              int cnt = 50;
              do
              {
                b_ok = true;
                ps[i] = &vec_pix[NextMt() % vec_pix.size()];
                for (int j = 0; j < i; j++)
                {
                  if (DIS2(*(ps[i]->p), *(ps[j]->p)) < MIN_DIS_BETWEEN_POINTS2)
                  {
                    b_ok = false;
                    break;
                  }
                  //printf("dis[%d][%d] : %.5lf\n", i, j
                  //  , DIS2( *(ps[i]->p), *(ps[j]->p) ));
                }
              } while (!b_ok && --cnt >= 0);
              if (!b_ok)
              {
                assert(cnt2 >= 0);
                b_ok2 = false;
                break;
              }
            }
          } while (!b_ok2);
        }

        assert(DIS2(*(ps[0]->p), *(ps[1]->p)) >= MIN_DIS_BETWEEN_POINTS2 && DIS2(*(ps[0]->p), *(ps[2]->p)) >= MIN_DIS_BETWEEN_POINTS2 && DIS2(*(ps[1]->p), *(ps[2]->p)) >= MIN_DIS_BETWEEN_POINTS2);
        /*
#define DIFF_VEC_ARRAY(p1,p2) {(p1).x-(p2).x, (p1).y-(p2).y, (p1).z-(p2).z}
        double v1[3] = DIFF_VEC_ARRAY(*(ps[1]->p), *(ps[0]->p))
          , v2[3] = DIFF_VEC_ARRAY(*(ps[2]->p), *(ps[0]->p));
        double cross[3] = 
          { v1[1]*v2[2] - v1[2]*v2[1]
          , v1[2]*v2[0] - v1[0]*v2[2]
          , v1[0]*v2[1] - v1[1]*v2[0] };
        double l = 1.0 / sqrt(POW2(cross[0])+POW2(cross[1])+POW2(cross[2]));
        cross[0]*= l;
        cross[1]*= l;
        cross[2]*= l;
        pcl::PointXYZ v1( ps[1]->p->x - ps[0]->p->x
          , ps[1]->p->y - ps[0]->p->y
          , ps[1]->p->z - ps[0]->p->z ), p_v0 = ps[0]->p;
        */

        // find plane
        {
          pcl::PointXYZ v[4], cross;
          MINUS_3D(v[0], *(ps[1]->p), *(ps[0]->p));
          MINUS_3D(v[1], *(ps[2]->p), *(ps[0]->p));
          NORMALIZE_3D(v[0]);
          NORMALIZE_3D(v[1]);
          CROSS_3D(cross, v[0], v[1]);
          double len2 = LEN2(cross);
          if (len2 < 0.01)
          {
            k1++;
            continue;
          }
          else
          {
            double len = (cross.z < 0.0 ? -1.0 : 1.0) / sqrt(len2);
            cross.x *= len;
            cross.y *= len;
            cross.z *= len;
          }

          int cnt = 0;
          pcl::PointXYZ vtmp;
          // count points near the plane
          for (int i = vec_pix.size() - 1; i >= 0; i--)
          {
            MINUS_3D(vtmp, *vec_pix[i].p, *ps[0]->p);
            double dis = fabs(INNER_3D(vtmp, cross));
            if (dis < config.plane_ransac_th_error)
            {
              cnt++;
            }
          }
          if (cnt > best_cnt)
          {
            best_plane.set_plane(*ps[0]->p, cross, v[0]);
            best_cnt = cnt;
          }
        }
      }
//      best_plane.p_reg = &r;
      best_plane.warp(*p_img_col, p_seg);
      planes.push_back(best_plane);

      if( config.show_result ){
        char str[16];
        sprintf(str, "warp%d.bmp", int(planes.size() - 1));
        cv::imshow(config.result_save_path + str, best_plane.img_plane);
      }
    }
  }

  void draw_reg()
  {
    if( config.show_result ){
      cv::Mat img = p_img_col->clone();
      const cv::Scalar col(100, 100, 250);
      for (int i = planes.size() - 1; i >= 0; i--)
      {
        const tRegInfo *r = planes[i].p_reg;
        cv::rectangle(img, cv::Point(r->x1, r->y1), cv::Point(r->x2, r->y2), col);
      }
      cv::imshow("reg", img);
      cv::imwrite(config.result_save_path + "reg.bmp", img);
    }
  }

  void find_text(){
    cLabeling label;
    visualization_msgs::Marker marker;
    geometry_msgs::Point marker_point;
    tPickPose pick_pose;
    marker.header.frame_id = FRAME_CAMERA;
    marker.ns = "pick_markers";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.005;
    marker.scale.y = 0.01;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    pick_poses.clear();
    for (int i_plane = planes.size() - 1; i_plane >= 0; i_plane--){
      printf("** find_text %d **\n", i_plane);
      cPlane &plane = planes[i_plane];
      cv::Mat //img_gray = cv::Mat( plane.img_plane.size(), CV_8UC1 )
        img_bin// = cv::Mat( plane.img_plane.size(), CV_8UC1 )
        , img_label, img_result;
      /*
      cv::cvtColor( plane.img_plane, img_gray, cv::COLOR_BGR2GRAY);
      cv::adaptiveThreshold(img_gray,img_bin, 255
        , CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY
        , config.th_text_binary_neighbour, config.th_text_binary_adapt_mean);
      */
      binarize( plane.img_plane, img_bin, config.text_binarize_win_size
        , config.text_binarize_subtract );
//      cv::threshold(img_gray,img_bin, config.text_binarize_win_size, 255, CV_THRESH_BINARY);// | CV_THRESH_OTSU);
      label.ExecBin( img_bin, img_label);
      if( config.show_result ){
        label.CreateImageResult( img_label, img_result, true);
        char str[20];
        sprintf(str, "text_region%d.bmp", i_plane);
        cv::imshow(str, img_result);
        cv::imwrite(config.result_save_path + str, img_result);

        sprintf(str, "plane%d.bmp", i_plane);
        cv::imshow(str, plane.img_plane);
        cv::imwrite(config.result_save_path + str, plane.img_plane);
      }


      cPCA_2D pca;
      for(int k=label.reg_info.size()-1;k>=0;k--){
        const tRegInfo &r = label.reg_info[k];
        /* pix = 789, ratio = 1.739
           pix = 722, ratio = 1.400 */
        if( r.pix_num<1200 || r.pix_num>5000 ){
          if( r.pix_num>100 )
            printf("invalid text reg : pix_num = %d\n", r.pix_num);
          continue;
        }
        double ratio = double(r.x2-r.x1+1)/(r.y2-r.y1+1);
        if( ratio<1.0 )
          ratio = 1.0/ratio;
        if( ratio>3.0 ){
          printf("invalid text reg : ratio = %.3lf\n", ratio);
          continue;
        }

        cv::Mat img_filter;
        fill_region( &r, img_label, img_filter);

        double vx[2], vy[2];
        double min_x, min_y, max_x, max_y;
        double center[2];
        min_x = min_y = 9999999.0;
        max_x = max_y = -min_x;

        // find label direction using edges
        cv::Mat img_edge =cv::Mat(img_filter.size(), CV_8UC1, cv::Scalar(0));
        {
          std::vector<cv::Point2i> edges;
          const int step = img_filter.step;
          
          cv::Point2i pnt;
          for(int i=img_filter.rows-2;i>=0;i--){
            unsigned char *p = img_filter.data + img_filter.step * i
              , *p2 = img_edge.data +  + img_edge.step * i;
            for(int j=img_filter.cols-2;j>=0;j--){
              if( p[j]!=p[j+1] || p[j]!=p[j+step] ){
                pnt.x = j;
                pnt.y = i;
                edges.push_back(pnt);
                p2[j] = 255;
              }
            }
          }          
          for(int i=img_filter.rows-1;i>=0;i--){
            unsigned char *p = img_filter.data + img_filter.step*i
              , *p2 = img_edge.data + img_edge.step*i;
            if( p[0]>0 ){
              pnt.x = 0;
              pnt.y = i;
              edges.push_back(pnt);
              p2[0] = 255;
            }
            if( p[img_filter.cols-1]>0 ){
              pnt.x = img_filter.cols-1;
              pnt.y = i;
              edges.push_back(pnt);
              p2[img_filter.cols-1] = 255;
            }
          }
          for(int j=img_filter.cols-1;j>0;j--){
            unsigned char *p = img_filter.data + j
              , *p2 = img_edge.data + j;
            if( p[0]>0 ){
              pnt.x = j;
              pnt.y = 0;
              edges.push_back(pnt);
              p2[0] = 255;
            }
            int i2 = img_filter.step*(img_filter.rows-1);
            if( p[i2]>0 ){
              pnt.x = j;
              pnt.y = img_filter.rows-1;
              edges.push_back(pnt);
              p2[i2] = 255;
            }
          }
          // ransac
          const double TH_DIS = config.text_ransac_th_error;
          int best_cnt = 0;
          cv::Point2d best_v;
          cv::Point2i best_p;
          for(int k=config.text_ransac_repeat_time;k>0;k--){
            const cv::Point2i *p1 = &edges[NextMt() % edges.size()], *p2;
            int cnt = 0;
            cv::Point2d v;
            for(;;){
              p2 = &edges[NextMt() % edges.size()];
              v.x = p2->x-p1->x;
              v.y = p2->y-p1->y;
              double len2 = POW2(v.x)+POW2(v.y);
              if( len2>=25 ){
                double len = 1.0/sqrt(len2);
                v.x*= len;
                v.y*= len;
                break;
              }
              assert(++cnt<100);
            }
            cnt = 0;
            for(int i=edges.size()-1;i>=0;i--){
              const cv::Point2i &p = edges[i];
              double x = p.x - p1->x, y = p.y - p1->y
                , dis = fabs(x*v.y - y*v.x);
              if( dis<=TH_DIS ){
                cnt++;
              }
            }
            if( cnt>best_cnt ){
              best_cnt = cnt;
              best_v.x = v.x;
              best_v.y = v.y;
              best_p.x = p1->x;
              best_p.y = p1->y;
            }
          }
          assert( best_cnt>0 );
          //printf("cnt : %d / %d\n", best_cnt, int(edges.size()));

          {
            double len = 1.0/sqrt( POW2(best_v.x)+POW2(best_v.y) );
            vx[0] = best_v.x * len;
            vx[1] = best_v.y * len;
            vy[0] = -vx[1];
            vy[1] = vx[0];
            center[0] = best_p.x;
            center[1] = best_p.y;
          }
          for(int i=edges.size()-1;i>=0;i--){
            const cv::Point2i &p = edges[i];
            double x = p.x - center[0], y = p.y - center[1]
              , new_x = x*vx[0] + y*vx[1], new_y = x*vy[0] + y*vy[1];
            if( new_x>max_x ) max_x = new_x;
            if( new_x<min_x ) min_x = new_x;
            if( new_y>max_y ) max_y = new_y;
            if( new_y<min_y ) min_y = new_y;
          }
          assert( min_x < 99999.0 );
        }
        /*
        // find label's direction using PCA
        {
          pca.reset();
          for(int i=img_filter.rows-1;i>=0;i--){
            unsigned char *p = img_filter.data + img_filter.step * i;
            for(int j=img_filter.cols-1;j>=0;j--){
              if( p[j] ){
                pca.n++;
                pca.nx+= j;
                pca.ny+= i;
                pca.xx+= j*j;
                pca.yy+= i*i;
                pca.xy+= i*j;
              }
            }
          }
          pca.run();
          vx[0] = pca.eig_v->data.db[0];
          vx[1] = pca.eig_v->data.db[2];
          vy[0] = pca.eig_v->data.db[1];
          vy[1] = pca.eig_v->data.db[3];
          
          for(int i=img_filter.rows-1;i>=0;i--){
            const unsigned char *p = img_filter.data + img_filter.step * i;
            for(int j=img_filter.cols-1;j>=0;j--){
              if( p[j] ){
                const double x = j - pca.mean[0], y = i - pca.mean[1]
                  , new_x = x*vx[0] + y*vx[1], new_y = x*vy[0] + y*vy[1];
                if( new_x>max_x ) max_x = new_x;
                if( new_x<min_x ) min_x = new_x;
                if( new_y>max_y ) max_y = new_y;
                if( new_y<min_y ) min_y = new_y;
              }
            }
          }
          center[0] = pca.mean[0];
          center[1] = pca.mean[1];
        }
        */
//        assert( max_x-min_x >= max_y-min_y );
        const int *r_xy = &r.x1;
        #define CAL_CORENER(x,y,axis) ( x*vx[axis] + y*vy[axis] + center[axis] + r_xy[axis] )
        cv::Point2f warp_src[4], warp_dst[4];
        warp_src[0].x = CAL_CORENER(min_x, min_y, 0);
        warp_src[0].y = CAL_CORENER(min_x, min_y, 1);
        warp_src[1].x = CAL_CORENER(max_x, min_y, 0);
        warp_src[1].y = CAL_CORENER(max_x, min_y, 1);
        warp_src[2].x = CAL_CORENER(max_x, max_y, 0);
        warp_src[2].y = CAL_CORENER(max_x, max_y, 1);
        warp_src[3].x = CAL_CORENER(min_x, max_y, 0);
        warp_src[3].y = CAL_CORENER(min_x, max_y, 1);
        
        printf("vx : %.3lf, %.3lf\n", vx[0], vx[1]);
        printf("vy : %.3lf, %.3lf\n", vy[0], vy[1]);
        printf("mx : %.3lf, %.3lf\n", min_x, max_x);
        printf("my : %.3lf, %.3lf\n", min_y, max_y);
        for(int i=0;i<4;i++){
          printf("warp_src[%d] : %.3f, %.3f\n", i, warp_src[i].x, warp_src[i].y);
        }

        warp_dst[0].x = 0.0f;
        warp_dst[0].y = 0.0f;
        warp_dst[1].x = max_x-min_x;
        warp_dst[1].y = 0.0f;
        warp_dst[2].x = max_x-min_x;
        warp_dst[2].y = max_y-min_y;
        warp_dst[3].x = 0.0f;
        warp_dst[3].y = max_y-min_y;
        cv::Mat img_text = cv::Mat( max_y-min_y, max_x-min_x, CV_8UC3);
        cv::Mat trans = cv::getPerspectiveTransform(warp_src, warp_dst);
        cv::warpPerspective(plane.img_plane, img_text, trans, img_text.size());

        {
          cv::Mat img_flip;
          cv::flip( img_text, img_flip, -1);
          cv::Mat *pi[2] = {&img_text, &img_flip};
          cv::Mat img_gray = cv::Mat( img_text.size(), CV_8UC1);
          int n_label = -1;
          for(int k=1;k>=0;k--){
            cv::cvtColor( *pi[k], img_gray, cv::COLOR_BGR2GRAY);
            const int MEAN = 140;
            for(int i=img_gray.rows-1;i>=0;i--){
              unsigned char *p = img_gray.data + i*img_gray.step;
              for(int j=img_gray.cols-1;j>=0;j--){
                int v = 3*(p[j]-MEAN) + MEAN;
                p[j] = v<0 ? 0 : (v>255 ? 255 : v);
              }
            }
            ocr.get_text(img_gray);

            for(int i=0;i<ocr.confs.size();i++){
              const char *str = ocr.texts[i].c_str();
              printf("text flip [%d]: %s, %.3f\n", k, str, ocr.confs[i]);
              if( str && str[0]=='M' && i<ocr.confs.size()-1 ){
                const char *str2 = ocr.texts[i+1].c_str();
                if( !str2 )
                  continue;
                switch(str2[0]){
                  case '1':
                  case 'i':
                  case 'I':
                  case 'l':
                  case 'L':
                    n_label = 1;
                    break;
                  case 'a':
                  case 'e':
                  case '2':
                    n_label = 2;
                    break;
                  case '3':
                  case 's':
                  case 'S':
                    n_label = 3;
                    break;
                  default:
                    printf("Unknown M : %c\n", str2[0]);
                }
              }
            }
            if( config.show_result ){
              char str[20];
              sprintf(str, "text_gray%d-%d.tif", i_plane, k);
              cv::imshow(str, img_gray);
              cv::imwrite(config.result_save_path + str, img_gray);
            }
            if( n_label>=0 ){
              double d = 1.0/255.0;
              marker.id = n_label;
              marker.color.r = cols[n_label-1][0] * d;
              marker.color.g = cols[n_label-1][1] * d;
              marker.color.b = cols[n_label-1][2] * d;
              marker.points.clear();
              plane.find_center(*p_seg->cloud, p_seg->img_label);
              marker_point.x = plane.center.x;
              marker_point.y = plane.center.y;
              marker_point.z = plane.center.z;
              marker.points.push_back(marker_point);
              marker_point.x+= plane.normal.x * -0.1;
              marker_point.y+= plane.normal.y * -0.1;
              marker_point.z+= plane.normal.z * -0.1;
              marker.points.push_back(marker_point);
              pick_markers.push_back(marker);

              {
                tf::Vector3 v(plane.normal.x,plane.normal.y,plane.normal.z)
                  , vx( 1.0, 0.0, 0.0)
                  , vn = vx.cross(v);
                vn.normalize();
                tf::Quaternion q(vn, acos(v.dot(vx)));
                pick_pose.pose.position.x = plane.center.x;
                pick_pose.pose.position.y = plane.center.y;
                pick_pose.pose.position.z = plane.center.z;
                pick_pose.pose.orientation.x = q.x();
                pick_pose.pose.orientation.y = q.y();
                pick_pose.pose.orientation.z = q.z();
                pick_pose.pose.orientation.w = q.w();

                char str[8];
                sprintf(str, "M%d", n_label);
                pick_pose.label = str;
                pick_poses.push_back(pick_pose);
              }
              break;
            }
          }
        }
        if( config.show_result ){
          char str[32];
          sprintf(str, "text%d.bmp", i_plane);
          cv::imshow(str, img_text);
          cv::imwrite(config.result_save_path + str, img_text);

          sprintf(str, "filter%d.bmp", i_plane);
          cv::line( img_filter
            , cv::Point( center[0], center[1] )
            , cv::Point(CAL_CORENER( max_x, 0, 0 )-r_xy[0], CAL_CORENER( max_x, 0, 1)-r_xy[1] )
            , cv::Scalar(150), 1);
          cv::line( img_filter
            , cv::Point( center[0], center[1] )
            , cv::Point(CAL_CORENER( 0, max_y, 0 )-r_xy[0], CAL_CORENER( 0, max_y, 1)-r_xy[1] )
            , cv::Scalar(150), 1);
          cv::imshow(str, img_filter);
          cv::imwrite(config.result_save_path + str, img_filter);

          sprintf(str, "filter_edge%d.bmp", i_plane);
          cv::imshow(str, img_edge);
        }
      }
    }
  }


/*  void ocr()
  {
    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();

    // Initialize tesseract to use English (eng) and the LSTM OCR engine.
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);

    // Set Page segmentation mode to PSM_AUTO (3)
    ocr->SetPageSegMode(tesseract::PSM_AUTO);

    for (int i = planes.size() - 1; i >= 0; i--)
    {
      cv::Mat &img = planes[i].img_plane;
      // Set image data
      ocr->SetImage(img.data, img.cols, img.rows, 3, img.step);
      ocr->Recognize(NULL);

      tesseract::ResultIterator *ri = ocr->GetIterator();
      if (ri != 0)
      {
        do
        {
          const char *symbol = ri->GetUTF8Text(tesseract::PageIteratorLevel::RIL_SYMBOL);
          if (symbol != 0)
          {
            float conf = ri->Confidence(tesseract::PageIteratorLevel::RIL_SYMBOL);
            printf("plane[%d] : %s\n", i, symbol);
            delete[] symbol;
          }
          else
          {
            printf("plane[%d] : no symbol\n", i);
          }
        } while ((ri->Next(tesseract::PageIteratorLevel::RIL_SYMBOL)));
      }
      else
      {
        printf("plane[%d] : no text found\n", i);
      }
      ocr->Clear();
    }
    delete ocr;
  }*/

  void create_plane_pointclouds()
  {
    const cv::Mat &img_label = p_seg->img_label;
    constexpr float bad_point = std::numeric_limits<float>::quiet_NaN();

/*    plane_pc.width = img_label.cols;
    plane_pc.height = img_label.rows;
    plane_pc.points.resize(plane_pc.width * plane_pc.height);
    plane_pc.is_dense = false;*/
    const std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>
        &pnts = p_seg->cloud->points;
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> pnts2 = plane_pc.points;
    pcl::PointXYZRGB p;
    for (int i = img_label.rows - 1; i >= 0; i--)
    {
      unsigned short *p_label = (unsigned short *)(img_label.data + img_label.step * i);
      int i2 = i * img_label.cols;
      for (int j = img_label.cols - 1; j >= 0; j--)
      {
        const int j2 = i2 + j;
        const pcl::PointXYZRGB &c = pnts[j2];
        //pcl::PointXYZRGB &p = pnts2[j2];
        if (!IS_VALID_POINT(c))
        {
        //  p.x = p.y = p.z = bad_point;
        }
        else
        {
          bool b_ok = false;
          for (int k = planes.size() - 1; k >= 0; k--)
          {
            if (p_label[j] == planes[k].p_reg->n_label)
            {
              const int *col = &cols[k % 6][0];
              b_ok = true;
              p.x = c.x;
              p.y = c.y;
              p.z = c.z;
              p.r = col[0];
              p.g = col[1];
              p.b = col[2];
              plane_pc.points.push_back(p);
              break;
            }
          }
        }
      }
      plane_pc.width = plane_pc.points.size();
      plane_pc.height = 1;
      plane_pc.is_dense = true;
    }
    {
      visualization_msgs::Marker marker;
      geometry_msgs::Point p;
      marker.header.frame_id = FRAME_CAMERA;
      marker.ns = "plane_frames";
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.005;
      marker.color.a = 1.0;
      for (int k = planes.size() - 1; k >= 0; k--)
      {
        const cPlane &plane = planes[k];
        const pcl::PointXYZ *ps = plane.corners;
        const int *col = &cols[k % 6][0];
        marker.color.r = col[0];
        marker.color.g = col[1];
        marker.color.b = col[2];
        marker.id = k;
        marker.points.clear();
        for (int i = 0; i < 4; i++)
        {
          const pcl::PointXYZ &p2 = plane.corners[i];
          p.x = p2.x;
          p.y = p2.y;
          p.z = p2.z;
          marker.points.push_back(p);
        }
        marker.points.push_back(marker.points[0]);
        plane_frames.push_back(marker);
      }
    }
  }

public:
  cSegment *p_seg;
  cv::Mat *p_img_col;
  std::vector<cPlane> planes;
  pcl::PointCloud<pcl::PointXYZRGB> plane_pc;
  std::vector<visualization_msgs::Marker> plane_frames;
  std::vector<visualization_msgs::Marker> pick_markers;
  std::vector<tPickPose> pick_poses;

  cSelectObject() : p_seg(NULL), p_img_col(NULL)
  {

  }

  void run(cSegment &seg, cv::Mat &img_col)
  {
    p_seg = &seg;
    p_img_col = &img_col;

    filter_region();
    ros::Time t = ros::Time::now();
    find_planes();
    ROS_INFO("find_planes() : %.3lf s", (ros::Time::now() - t).toSec());
    t = ros::Time::now();
    find_text();
    ROS_INFO("find_text() : %.3lf s", (ros::Time::now() - t).toSec());
    
    t = ros::Time::now();
    create_plane_pointclouds();
    ROS_INFO("create_plane_pointclouds() : %.3lf s", (ros::Time::now() - t).toSec());
    t = ros::Time::now();
    draw_reg();
    ROS_INFO("draw_reg() : %.3lf s", (ros::Time::now() - t).toSec());
  }
};

#endif