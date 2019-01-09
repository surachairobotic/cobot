#ifndef __CFINDLABEL_H__
#define __CFINDLABEL_H__

#include "cobot_pick/common.h"
#include "cobot_pick/config_find_object.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cobot_pick/cLabeling.h"
#include "cobot_pick/cSelectObject.h"
#include "cobot_pick/cOCR.h"

class cFindLabel{
private:
  cOCR ocr;

  // p1 + k1*v1 = p2 + k2*v2
  bool find_cross_point_2D( const cv::Point2d &p1, const cv::Point2d &v1
    , const cv::Point2d &p2, const cv::Point2d &v2
    , cv::Point2f &cross_p ){

    if( fabs(CROSS_2D( v1, v2)) < 0.3 )
      return false;
    
    cv::Point2d p0( p2.x-p1.x, p2.y-p1.y );
    double det = 1.0/(v1.x*-v2.y + v2.x*v1.y);
//    if( fabs(det) < 0.0001 )
//      return false;
    double k1 = det*(-v2.y*p0.x + v2.x*p0.y)
      , k2 = det*(-v1.y*p0.x + v1.x*p0.y);
    assert( fabs(p1.x + k1*v1.x - p2.x - k2*v2.x)<0.0001 
      && fabs(p1.y + k1*v1.y - p2.y - k2*v2.y)<0.0001 );
    return true;
  }

  void remove_edges(const std::vector<cv::Point2i> &edges
      std::vector<cv::Point2i> &new_edges
      , const cv::Point2d &bp, const cv::Point2d &bv
      , const double dis){
    new_edges.clear();
    new_edges.reserve(edges.size()/2);
    for(int i=edges.size()-1;i>=0;i--){
      cv::Point2i p;
      const cv::Point2i &e = edges[i];
      p.x = e.x - bp.x;
      p.y = e.y - bp.y;
      double cross = fabs( p.x*bv.y - p.y*bv.x );
      if( cross > dis ){
        new_edges.push_back(e);
      }
    }
    printf("new_edges : %lu / %lu\n", new_edges.size(), edges.size());
  }

  void ransac_2d(const std::vector<cv::Point2i> &edges
      , cv::Point2d &best_p, cv::Point2d &best_v){
    const double TH_DIS = config.text_ransac_th_error;
    int best_cnt = 0;
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
  }

  void ransac_3d(const tRegInfo &r
        , const pcl::PointCloud<pcl::PointXYZRGB> &cloud
        , cPlane &best_plane ){

    // min distance between points that are picked up for ransac
    const double MIN_DIS_BETWEEN_POINTS2 = POW2(0.01);
    const std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &pnts = cloud.points;
    std::vector<tPoint2i> vec_pix;
    const int w = cloud.width, h = cloud.height;

    tPoint2i p2i;
    vec_pix.reserve(10000);
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
    int best_cnt = 0;
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
  }
public:
  std::vector<visualization_msgs::Marker> pick_markers;
  std::vector<tPickPose> pick_poses;

  cFindLabel(){}

  void run(const cv::Mat &img_col, const pcl::PointCloud<pcl::PointXYZRGB> &cloud){
    cLabeling label;
    cv::Mat img_bin = cv::Mat( img_col.size(), CV_8UC1), img_label, img_result;
    visualization_msgs::Marker marker, marker_frame;

    binarize( img_col, img_bin, config.text_binarize_win_size
        , config.text_binarize_subtract);
    label.ExecBin( img_bin, img_label);

    geometry_msgs::Point marker_point;
    tPickPose pick_pose;
    marker.header.frame_id = marker_frame.header.frame_id = FRAME_CAMERA;
    marker.ns = "pick_markers";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.005;
    marker.scale.y = 0.01;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;

    marker_frame.ns = "pick_frames";
    marker_frame.type = visualization_msgs::Marker::LINE_STRIP;
    marker_frame.scale.x = 0.005;
    marker_frame.scale.y = 0.005;
    marker_frame.scale.z = 0.005;
    marker_frame.color.a = 1.0;

    pick_poses.clear();

    if( config.show_result ){      
      label.CreateImageResult( img_label, img_result, true);
      char str[20] = "region.bmp";
      cv::imwrite(config.result_save_path + str, img_result);
    }

    for(int ir=label.reg_info.size()-1;ir>=0;ir--){
      const tRegInfo &r = label.reg_info[ir];
      /* pix = 789, ratio = 1.739
          pix = 722, ratio = 1.400 */
      if( r.pix_num<500 || r.pix_num>3000 ){
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
      if( config.show_result ){
        cv::rectangle(img_result, cv::Point(r.x1, r.y1)
          , cv::Point(r.x2, r.y2), cv::Scalar(30,30,255));
      }

      cv::Mat img_filter;
      fill_region( &r, img_label, img_filter);

      double center[2];
      
      double vx[2], vy[2];
      double min_x, min_y, max_x, max_y;
      cv::Point2f warp_src[4], warp_dst[4];
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
        cv::Point2d best_v[4], best_p[4];
        std::vector<cv::Point2i> new_edges, *p_edges[2] = {&edges, &new_edges};

        for(int i=0;i<4;i++){
          ransac_2d( *p_edges[i%2], best_p[i], best_v[i]);
          if( i<3 )
            remove_edges( *p_edges[i%2], *p_edges[(i+1)%2]
              , best_p[i], best_v[i], 2.0 );
        }

        {

          const int n = [[0, 2], [0, 3], [1, 3], [1, 2]];
          bool b_ok = true;
          for(int i=0;i<4;i++){
            if( !find_cross_point_2D( best_p[n[i][0]], best_v[n[i][0]]
              , best_p[n[i][1]], best_v[n[i][1]]
              , warp_src[i] ) ){
              b_ok = false;
              break;
            }
          }
          if( !b_ok )
            continue;
          {
            double x = sqrt( DIS2_2D(warp_src[0], warp_src[1]))
              , y = sqrt( DIS2_2D(warp_src[1], warp_src[2]))
            if( x<10 || y<10 )
              continue;
            warp_dst[0].x = 0.0f;
            warp_dst[0].y = 0.0f;
            warp_dst[1].x = x;
            warp_dst[1].y = 0.0f;
            warp_dst[2].x = x;
            warp_dst[2].y = y;
            warp_dst[3].x = 0.0f;
            warp_dst[3].y = y;
          }

        // find 1st line
/*        ransac_2d(edges, best_p[0], best_v[0]);
        // remove edges for find 2nd line
        {
          edges2.reserve(edges.size()*0.5);
          double min_inner = 9999.0, max_inner = -9999.0;
          for(int i=edges.size()-1;i>=0;i--){
            cv::Point2i p;
            const cv::Point2i &e = edges[i];
            const cv::Point2d &bp = best_p[0], &bv = best_v[0];
            p.x = e.x - bp.x;
            p.y = e.y - bp.y;
            double cross = fabs( p.x*bv.y - p.y*bv.x );
            if( cross > 2.0 ){
              edges2.push_back(e);
            }
            else{
              double inner = p.x*bv.x + p.y*bv.y;
              if( inner < min_inner ){
                min_inner = inner;
                warp_src[0].x = e.x;
                warp_src[0].y = e.y;
              }
              if( inner > max_inner ){
                max_inner = inner;
                warp_src[1].x = e.x;
                warp_src[1].y = e.y;
              }
            }
          }
          printf("edges2 : %lu / %lu\n", edges2.size(), edges.size());
        }
        // find 2nd line
        ransac_2d(edges2, best_p[1], best_v[1]);
        {
          if( fabs(best_v[0].x)>0.1  ){
            if( best_v[0].x*best_v[1].x<0.0 ){
              best_v[1].x*=-1.0;
              best_v[1].y*=-1.0;
            }
          }
          else{
            if( best_v[0].y*best_v[1].y<0.0 ){
              best_v[1].x*=-1.0;
              best_v[1].y*=-1.0;
            }
          }
          double min_inner = 9999.0, max_inner = -9999.0;
          for(int i=edges2.size()-1;i>=0;i--){
            cv::Point2i p;
            const cv::Point2i &e = edges2[i];
            const cv::Point2d &bp = best_p[1], &bv = best_v[1];
            p.x = e.x - bp.x;
            p.y = e.y - bp.y;
            if( fabs( p.x*bv.y - p.y*bv.x ) < 2.0 ){
              double inner = p.x*bv.x + p.y*bv.y;
              if( inner < min_inner ){
                min_inner = inner;
                warp_src[3].x = e.x;
                warp_src[3].y = e.y;
              }
              if( inner > max_inner ){
                max_inner = inner;
                warp_src[2].x = e.x;
                warp_src[2].y = e.y;
              }
            }
          }
        }
        {
          double x = sqrt( POW2(warp_src[0].x-warp_src[1].x)
            +POW2(warp_src[0].y-warp_src[1].y) )
            , y = sqrt( POW2(warp_src[0].x-warp_src[3].x)
            +POW2(warp_src[0].y-warp_src[3].y) );
          warp_dst[0].x = 0.0f;
          warp_dst[0].y = 0.0f;
          warp_dst[1].x = x;
          warp_dst[1].y = 0.0f;
          warp_dst[2].x = x;
          warp_dst[2].y = y;
          warp_dst[3].x = 0.0f;
          warp_dst[3].y = y;
        }

        for(int i=3;i>=0;i--){
          warp_src[i].x+= r.x1;
          warp_src[i].y+= r.y1;
        }*/
        
        

        /*
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

        const int *r_xy = &r.x1;
        #define CAL_CORENER(x,y,axis) ( x*vx[axis] + y*vy[axis] + center[axis] + r_xy[axis] )
        
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
        */
      }
      if( warp_dst[2].y< 10 || warp_dst[2].x < 10 )
        continue;
/*      for(int i=0;i<4;i++){
        printf("warp src[%d]: %.3f, %.3f\n", i, warp_src[i].x, warp_src[i].y);
      }
      for(int i=0;i<4;i++){
        printf("warp dst[%d]: %.3f, %.3f\n", i, warp_dst[i].x, warp_dst[i].y);
      }*/
      
      cv::Mat img_text = cv::Mat( warp_dst[2].y, warp_dst[2].x, CV_8UC3);
      cv::Mat trans = cv::getPerspectiveTransform(warp_src, warp_dst);
      cv::warpPerspective(img_col, img_text, trans, img_text.size());

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
            printf("text flip [%d][%d]: %s, %.3f\n", ir, k, str, ocr.confs[i]);
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
            sprintf(str, "text_gray%d-%d.tif", ir, k);
            cv::imshow(str, img_gray);
            cv::imwrite(config.result_save_path + str, img_gray);
          }

          cPlane plane(&r);
          ransac_3d( r, cloud, plane );

          if( n_label>=0 ){
            double d = 1.0/255.0;
            marker.id = marker_frame.id = n_label;
            marker.color.r = marker_frame.color.r = cols[n_label-1][0] * d;
            marker.color.g = marker_frame.color.g = cols[n_label-1][1] * d;
            marker.color.b = marker_frame.color.b = cols[n_label-1][2] * d;
            marker.points.clear();
            marker_frame.points.clear();
            plane.find_center(cloud, img_label);
            marker_point.x = plane.center.x;
            marker_point.y = plane.center.y;
            marker_point.z = plane.center.z;
            marker.points.push_back(marker_point);
            marker_point.x+= plane.normal.x * -0.1;
            marker_point.y+= plane.normal.y * -0.1;
            marker_point.z+= plane.normal.z * -0.1;
            marker.points.push_back(marker_point);
            pick_markers.push_back(marker);

            for(int i=0;i<5;i++){
              const pcl::PointXYZ &f = plane.frames[i%4];
              marker_point.x = f.x;
              marker_point.y = f.y;
              marker_point.z = f.z;
              marker_frame.points.push_back(marker_point);
            }
            pick_markers.push_back(marker_frame);

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
        sprintf(str, "text%d.bmp", ir);
        cv::imshow(str, img_text);
        cv::imwrite(config.result_save_path + str, img_text);

        sprintf(str, "filter%d.bmp", ir);
/*        cv::line( img_filter
//          , cv::Point( center[0], center[1] )
//          , cv::Point(CAL_CORENER( max_x, 0, 0 )-r_xy[0], CAL_CORENER( max_x, 0, 1)-r_xy[1] )
          , cv::Point( warp_src[0].x - r.x1, warp_src[0].y - r.y1 )
          , cv::Point( warp_src[1].x - r.x1, warp_src[1].y - r.y1 )
          , cv::Scalar(150), 1);
        cv::line( img_filter
//          , cv::Point( center[0], center[1] )
//          , cv::Point(CAL_CORENER( 0, max_y, 0 )-r_xy[0], CAL_CORENER( 0, max_y, 1)-r_xy[1] )
          , cv::Point( warp_src[0].x - r.x1, warp_src[0].y - r.y1 )
          , cv::Point( warp_src[3].x - r.x1, warp_src[3].y - r.y1 )
          , cv::Scalar(150), 1);*/
        for(int i=0;i<4;i++){
          cv::line( img_result
          , warp_src[i]
          , warp_src[(i+1)%4]
          , cv::Scalar(200,200,200), 1);
        }
        cv::imshow(str, img_filter);
        cv::imwrite(config.result_save_path + str, img_filter);

        sprintf(str, "filter_edge%d.bmp", ir);
        cv::imshow(str, img_edge);
      }
    }
    if( config.show_result ){
      cv::imshow( "result.bmp", img_result);
    }
  }

};

#endif