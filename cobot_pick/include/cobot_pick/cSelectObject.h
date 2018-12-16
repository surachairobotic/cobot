
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
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <visualization_msgs/Marker.h>

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
  pcl::PointXYZ point, normal, x_axis, y_axis;
  tRegInfo *p_reg;
  //  double corners_2d[4][2];
  pcl::PointXYZ corners[4];
  cv::Mat img_plane;

  cPlane() {}

  void set_plane(const pcl::PointXYZRGB &_p, const pcl::PointXYZ &_normal, const pcl::PointXYZ &_x_axis)
  {
    point.x = _p.x;
    point.y = _p.y;
    point.z = _p.z;
    normal = _normal;
    NORMALIZE_3D(x_axis, _x_axis);
    CROSS_3D(y_axis, normal, x_axis);
  }

  void warp(const cv::Mat &img_col, const cSegment *p_seg)
  {
    // create image
    cv::Mat img = cv::Mat(img_col, cv::Rect(p_reg->x1, p_reg->y1, p_reg->x2 - p_reg->x1 + 1, p_reg->y2 - p_reg->y1 + 1)).clone();
    const cv::Mat &img_label = p_seg->img_label;
    for (int i = img.rows - 1; i >= 0; i--)
    {
      unsigned short *p_label = (unsigned short *)(img_label.data + img_label.step * (i + p_reg->y1)) + p_reg->x1;
      unsigned char *p = (unsigned char *)(img.data + img.step * i);
      for (int j = img.cols - 1; j >= 0; j--)
      {
        if (p_label[j] != p_reg->n_label)
        {
          int j2 = j * 3;
          p[j2] = p[j2 + 1] = p[j2 + 2] = 0;
        }
      }
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
    printf("warp src 1: %d, %d\n", p_reg->x1, p_reg->y1);
    printf("warp src 2: %d, %d\n", p_reg->x2, p_reg->y2);
    for (int i = 3; i >= 0; i--)
    {
      cConvert3D::get_vector(warp_src[i].x + p_reg->x1, warp_src[i].y + p_reg->y1, corners[i]);
      // find corner 3d points on the plane
      pcl::PointXYZ &p = corners[i];
      const double k = np / INNER_3D(normal, p);
      p.x *= k;
      p.y *= k;
      p.z *= k;
      printf("pnt : %.3f, %.3f, %.3f\n", p.x, p.y, p.z);

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
/*    {
      static int cc = 0;
      char str[16];
      sprintf(str, "img%d", cc);
      cv::imshow(str, img);
      cc++;
    }*/
  }
};

class cSelectObject
{
private:
  std::vector<int> reg_index;

  void filter_region()
  {
    reg_index.clear();
    for (int i = p_seg->label.reg_info.size() - 1; i >= 0; i--)
    {
      const tRegInfo &r = p_seg->label.reg_info[i];
      if (r.pix_num < config.reg_min_pix || r.pix_num > config.reg_max_pix)
        continue;

      const int dx = r.x2 - r.x1 + 1, dy = r.y2 - r.y1 + 1;
      double ratio = dx > dy ? dx / double(dy) : dy / double(dx);
      if (ratio < config.reg_min_ratio || ratio > config.reg_max_ratio)
        continue;
      reg_index.push_back(i);
    }
    ROS_INFO("filter reg : %d / %d", (int)reg_index.size(), (int)p_seg->label.reg_info.size());
  }

  struct tPoint2i
  {
    int i, j, u, v;
    pcl::PointXYZRGB *p;
    tPoint2i() {}
    tPoint2i(int _i, int _j, pcl::PointXYZRGB *_p) : i(_i), j(_j), p(_p) {}
  };
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
      cPlane best_plane;
      for (int k1 = config.ransac_repeat_time - 1; k1 >= 0; k1--)
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
            if (dis < config.ransac_th_error)
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
      best_plane.p_reg = &r;
      best_plane.warp(*p_img_col, p_seg);
      planes.push_back(best_plane);

      char str[16];
      sprintf(str, "warp%d.bmp", int(planes.size() - 1));
      cv::imshow(save_path + str, best_plane.img_plane);
//      cv::imwrite(save_path + str, best_plane.img_plane);
    }
  }

  void draw_reg()
  {
    cv::Mat img = p_img_col->clone();
    const cv::Scalar col(100, 100, 250);
    for (int i = planes.size() - 1; i >= 0; i--)
    {
      const tRegInfo *r = planes[i].p_reg;
      cv::rectangle(img, cv::Point(r->x1, r->y1), cv::Point(r->x2, r->y2), col);
    }
    cv::imshow("reg", img);
    cv::imwrite(save_path + "reg.bmp", img);
  }

  void find_text(){
    cLabeling label;
    for (int k = planes.size() - 1; k >= 0; k--){
      cv::Mat img_gray = cv::Mat( planes[k].img_plane.size(), CV_8UC1 )
        , img_bin = cv::Mat( planes[k].img_plane.size(), CV_8UC1 )
        , img_label, img_result;
      cv::cvtColor( planes[k].img_plane, img_gray, cv::COLOR_BGR2GRAY);
      cv::adaptiveThreshold(img_gray,img_bin, 255
        , CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY, config.th_text_binary, 0); 
//      cv::threshold(img_gray,img_bin, config.th_text_binary, 255, CV_THRESH_BINARY);// | CV_THRESH_OTSU);
      label.ExecBin( img_bin, img_label);
      label.CreateImageResult( img_label, img_result, true);
      {
        char str[16];
        sprintf(str, "text%d", k);
        cv::imshow(str, img_result);
        sprintf(str, "gray%d", k);
        cv::imshow(str, img_gray);
      }
/*      for(int i=img.rows-1;i>=0;i--){
        for(int j=img.cols-1;j>=0;j--){
          
        }
      }*/
    }
  }

  void ocr()
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
  }

  void create_plane_pointclouds()
  {
    const cv::Mat &img_label = p_seg->img_label;
    constexpr float bad_point = std::numeric_limits<float>::quiet_NaN();

    const int cols[6][3] = {
        {255, 50, 50},
        {50, 255, 50},
        {50, 50, 255},
        {255, 250, 50},
        {50, 255, 250},
        {250, 50, 255}};

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
      marker.header.frame_id = "my_frame";
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

  cSelectObject() : p_seg(NULL), p_img_col(NULL)
  {

    unsigned int aa[128];
    srand((int)ros::Time::now().toNSec());
    for (int i = 0; i < 128; i++)
    {
      aa[i] = (unsigned int)rand();
    }
    InitMtEx(aa, 128);
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
/*    ocr();
    ROS_INFO("ocr() : %.3lf s", (ros::Time::now() - t).toSec());
    t = ros::Time::now();
    create_plane_pointclouds();
    ROS_INFO("create_plane_pointclouds() : %.3lf s", (ros::Time::now() - t).toSec());
    t = ros::Time::now();
    */
    draw_reg();
    ROS_INFO("draw_reg() : %.3lf s", (ros::Time::now() - t).toSec());
  }
};

#endif