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


void print_reg(const tRegInfo &r){
//  printf("reg : (%d, %d), (%d, %d)\n", r.x1, r.y1, r.x2, r.y2);
}


class cFindLabel{
private:
  cOCR ocr;

  // p1 + k1*v1 = p2 + k2*v2
  bool find_cross_point_2D( const cv::Point2d &p1, const cv::Point2d &v1
    , const cv::Point2d &p2, const cv::Point2d &v2
    , cv::Point2f &cross_p ){

    if( fabs(CROSS_2D( v1, v2)) < 0.3 )
      return false;
    
    const cv::Point2d p0( p2.x-p1.x, p2.y-p1.y );
    const double det = 1.0/(v1.x*-v2.y + v2.x*v1.y);
//    if( fabs(det) < 0.0001 )
//      return false;
    const double k1 = det*(-v2.y*p0.x + v2.x*p0.y)
      , k2 = det*(-v1.y*p0.x + v1.x*p0.y);
    cross_p.x = p1.x + k1*v1.x;
    cross_p.y = p1.y + k1*v1.y;

    assert( fabs(cross_p.x - p2.x - k2*v2.x)<0.0001 
      && fabs(cross_p.y - p2.y - k2*v2.y)<0.0001 );
    
    return true;
  }

  void remove_edges(const std::vector<cv::Point2i> &edges
      , std::vector<cv::Point2i> &new_edges
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
//    printf("new_edges : %lu / %lu\n", new_edges.size(), edges.size());
  }
  
  void draw_edges( const std::vector<cv::Point2i> &edges, const cv::Size size, const char *fname ){
    cv::Mat img = cv::Mat( size, CV_8UC1, cv::Scalar(0) );
    for(int i=edges.size()-1;i>=0;i--){
      img.data[ edges[i].y * img.step + edges[i].x ] = 255;
    }
    cv::imwrite(config.result_save_path + fname, img);
  }

  bool ransac_2d(const std::vector<cv::Point2i> &edges
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
        if( len2>=9 ){
          double len = 1.0/sqrt(len2);
          v.x*= len;
          v.y*= len;
          break;
        }
        if(++cnt>100){
          ROS_INFO("ransac2d failed : edges = %d\n", (int)edges.size());
          return false;
        }
//        assert(++cnt<100);
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
    return best_cnt>0;
  }

  bool ransac_3d(const tRegInfo &r
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
    
    if( vec_pix.size()<10 )
      return false;

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
                //ROS_INFO("dis[%d][%d] : %.5lf\n", i, j
                //  , DIS2( *(ps[i]->p), *(ps[j]->p) ));
              }
            } while (!b_ok && --cnt >= 0);
            if (!b_ok)
            {
              if( --cnt2>=0 ){
                b_ok2 = false;
                break;
              }
              else
                return false;
              /*
              cv::Mat img( h, w, CV_8UC1 );
              for(int i=h-1;i>=0;i--){
                unsigned char *p = img.data + img.step * i;
                for(int j=w-1;j>=0;j--){
                  p[j] = IS_VALID_POINT(pnts[i*w + j]) ? 255 : 0;
                }
              }
              cv:imwrite(config.result_save_path + "valid_pc.bmp", img);
              assert(--cnt2 >= 0);
              b_ok2 = false;
              break;*/
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
          best_plane.set_plane(*ps[0]->p, cross);//, v[0]);
          best_cnt = cnt;
        }
      }
    }
    return true;
  }
public:
  std::vector<visualization_msgs::Marker> pick_markers, frame_markers
    , place_markers, basket_markers;
  std::vector<cPlane> planes;
  tPickPose pick_pose;

  cFindLabel(){
    int n = 8;
    pick_markers.resize(n);
    frame_markers.resize(n);
    place_markers.resize(n);
    basket_markers.resize(n);
    
    std::vector<visualization_msgs::Marker>* ms[4] = {
      &pick_markers, 
      &frame_markers, 
      &place_markers, 
      &basket_markers
    };
    const double d = 1.0/255.0;
    for(int j=0;j<4;j++){
      std::string ns, frame_id;
      int type;
      if( j%2==0 ){
        ns = NS_PICK_ARROW;
        type = visualization_msgs::Marker::ARROW;
        frame_id = FRAME_WORLD;
      }
      else{
        ns = NS_LABEL_FRAME;
        type = visualization_msgs::Marker::LINE_STRIP;
        frame_id = j==3 ? FRAME_WORLD : FRAME_CAMERA;
      }
      for(int i=0;i<n;i++){
        visualization_msgs::Marker &m = (*ms[j])[i];
        m.id = i;
        m.header.frame_id = frame_id;
        m.ns = ns;
        m.type = type;
/*        m.color.r = cols[i][0] * d;
        m.color.g = cols[i][1] * d;
        m.color.b = cols[i][2] * d; */
        m.color.a = 1.0;
        if( j%2==0 ){
          m.scale.x = 0.005;
          m.scale.y = 0.01;// arrow width
          m.scale.z = 0.02;// arrow height
        }
        else{
          m.scale.x = 0.005;
          m.scale.y = 0.005;
          m.scale.z = 0.005;
        }
      }
    }
  }
  
  
  void set_frame_id_camera(const std::string &frame_id){
    std::vector<visualization_msgs::Marker>* ms[2] = { &frame_markers , &basket_markers};
    for(int j=0;j<1;j++){
      for(int i=0;i<frame_markers.size();i++){
        visualization_msgs::Marker &m = (*ms[j])[i];
        m.header.frame_id = frame_id;
      }
    }
  }

  void run(const cv::Mat &img_col, const pcl::PointCloud<pcl::PointXYZRGB> &cloud){
    cLabeling label;
    cv::Mat img_bin = cv::Mat( img_col.size(), CV_8UC1), img_label, img_result;
    int cnt_sample = 0;
    double max_plane_z = 0.0;
    int pick_plane_id = -1;
    int n_pick_marker = 0;

    planes.clear();
    binarize( img_col, img_bin, config.text_binarize_win_size
        , config.text_binarize_subtract);
    label.ExecBin( img_bin, img_label);

    if( config.show_result || config.save_result ){      
      label.CreateImageResult( img_label, img_result, true);
      char str[20] = "region.bmp";
      if( config.save_result )
        cv::imwrite(config.result_save_path + str, img_result);
    }

    for(int ir=label.reg_info.size()-1;ir>=0;ir--){
      const tRegInfo &r = label.reg_info[ir];
      ROS_INFO("reg [%d]: (%d, %d), (%d, %d)", ir, r.x1, r.y1, r.x2, r.y2);
      if( r.pix_num<config.min_label_pix_num || r.pix_num>config.max_label_pix_num ){
        if( r.pix_num>100 ){
          ROS_INFO("* invalid text reg [%d] : pix_num = %d\n", ir, r.pix_num);
          print_reg(r);
        }
        continue;
      }
      double ratio = double(r.x2-r.x1+1)/(r.y2-r.y1+1);
      if( ratio<1.0 )
        ratio = 1.0/ratio;
      if( ratio>config.max_label_ratio || ratio<config.min_label_ratio ){
        ROS_INFO("* invalid text reg [%d] : ratio = %.3lf\n", ir, ratio);
        print_reg(r);
        continue;
      }
      // exclude non black & white region
      {
        unsigned int sum = 0;
        for(int i=r.y1;i<=r.y2;i++){
          const unsigned char *pi = img_col.data + img_col.step * i;
          const unsigned short *pl = (const unsigned short*)(img_label.data + img_label.step * i);
          for(int j=r.x1;j<=r.x2;j++){
            if( pl[j]==r.n_label ){
              const unsigned char *p = pi + j*3;
              sum+= POW2(p[0]-p[1]) + POW2(p[2]-p[1]) + POW2(p[0]-p[2]);
            }
          }
        }
        double c = double(sum)/r.pix_num;
        if( c>config.th_region_color ){
          ROS_INFO("* invalid reg color [%d]: %.3lf\n", ir, c );
          print_reg(r);
          continue;
        }
      }
      if( config.show_result || config.save_result ){
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
        for(int i=img_filter.rows-2;i>0;i--){
          unsigned char *p = img_filter.data + img_filter.step * i
            , *p2 = img_edge.data +  + img_edge.step * i;
          for(int j=img_filter.cols-2;j>0;j--){
            if( p[j] && ( !p[j+1] || !p[j-1] || !p[j-step] || !p[j+step] )){
//            if( p[j]!=p[j+1] || p[j]!=p[j+step] ){
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
          if( p[0] ){
            pnt.x = 0;
            pnt.y = i;
            edges.push_back(pnt);
            p2[0] = 255;
          }
          if( p[img_filter.cols-1] ){
            pnt.x = img_filter.cols-1;
            pnt.y = i;
            edges.push_back(pnt);
            p2[img_filter.cols-1] = 255;
          }
        }
        for(int j=img_filter.cols-1;j>0;j--){
          unsigned char *p = img_filter.data + j
            , *p2 = img_edge.data + j;
          if( p[0] ){
            pnt.x = j;
            pnt.y = 0;
            edges.push_back(pnt);
            p2[0] = 255;
          }
          int i2 = img_filter.step*(img_filter.rows-1);
          if( p[i2] ){
            pnt.x = j;
            pnt.y = img_filter.rows-1;
            edges.push_back(pnt);
            p2[i2] = 255;
          }
        }
        
        // ransac
        cv::Point2d best_v[4], best_p[4];
        std::vector<cv::Point2i> edges_backup = edges;
        std::vector<cv::Point2i> new_edges, *p_edges[2] = {&edges, &new_edges};
        {
          bool b_ok = true;
          char fname[16];
          for(int i=0;i<4;i++){
            if( !ransac_2d( *p_edges[i%2], best_p[i], best_v[i]) ){
              b_ok = false;
              ROS_INFO("* ransac2d failed [%d]\n", ir);
              break;
            }
            if( config.save_result ){
              sprintf(fname, "edges%03d_%d.jpg", ir, i);
              draw_edges( *p_edges[i%2], img_filter.size(), fname );
            }
            if( i<3 ){
              remove_edges( *p_edges[i%2], *p_edges[(i+1)%2]
                , best_p[i], best_v[i], config.dis_remove_edge );
              if( p_edges[(i+1)%2]->size()<4 ){
                b_ok = false;
                ROS_INFO("* too few edges [%d]\n", ir);
                break;
              }
            }
          }
          if( !b_ok ){
            print_reg(r);
/*            cv::imshow("img_edge", img_edge);
            cv::imshow("img_filter", img_filter);
            cv::waitKey(0);*/
            continue;
          }
        }

        {

          const int n[4][2] = {{0, 2}, {0, 3}, {1, 3}, {1, 2}};
          bool b_ok = true;
          for(int i=0;i<4;i++){
            if( !find_cross_point_2D( best_p[n[i][0]], best_v[n[i][0]]
              , best_p[n[i][1]], best_v[n[i][1]]
              , warp_src[i] ) ){
              
              ROS_INFO("* invalid line %d (%d,%d) : %.3lf, %.3lf / %.3lf, %.3lf\n"
                , ir, n[i][0], n[i][1]
                , best_v[n[i][0]].x, best_v[n[i][0]].y
                , best_v[n[i][1]].x, best_v[n[i][1]].y );
//              assert( ir!=333 );
              b_ok = false;
              
              if( config.save_result ){
                char str[32];
                sprintf(str, "line_filter_%d.bmp", ir);
                cv::imwrite(config.result_save_path + str, img_filter);
                
                memset( img_filter.data, 0, img_filter.rows * img_filter.cols );
                for(int i=edges_backup.size()-1;i>=0;i--){
                  const cv::Point2i &e = edges_backup[i];
                  img_filter.data[ int(e.y+0.5) * img_filter.cols 
                    + int(e.x + 0.5)] = 255;
                }
                sprintf(str, "line_edges_%d.bmp", ir);
                cv::imwrite(config.result_save_path + str, img_filter);
              }
              break;
              
            }
          }
          if( !b_ok )
            continue;
          {
            double x = sqrt( DIS2_2D(warp_src[0], warp_src[1]))
              , y = sqrt( DIS2_2D(warp_src[1], warp_src[2]));
            if( x<config.min_label_size_x || y<config.min_label_size_y ){
              ROS_INFO("* invalid label size [%d] : %.3lf, %.3lf\n"
                , ir, x, y);
              for(int i=0;i<4;i++){
                ROS_INFO("warp src[%d]: %.3f, %.3f\n"
                  , i, warp_src[i].x, warp_src[i].y);
              }
              continue;
            }
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
          }

          // check label flip
          {
            cv::Point2d v1(warp_src[1].x-warp_src[0].x, warp_src[1].y-warp_src[0].y )
              , v2(warp_src[2].x-warp_src[1].x, warp_src[2].y-warp_src[1].y );
            if( CROSS_2D( v1, v2 )<0.0 ){
              cv::Point2d p;
              p = warp_src[0];
              warp_src[0] = warp_src[3];
              warp_src[3] = p;

              p = warp_src[1];
              warp_src[1] = warp_src[2];
              warp_src[2] = p;
            }
          }
        }
      }
      
      cv::Mat img_text = cv::Mat( warp_dst[2].y, warp_dst[2].x, CV_8UC3);
      cv::Mat trans = cv::getPerspectiveTransform(warp_src, warp_dst);
      cv::warpPerspective(img_col, img_text, trans, img_text.size());

      {
        cv::Mat img_flip;
        cv::flip( img_text, img_flip, -1);
        cv::Mat *pi[2] = {&img_text, &img_flip};
        cv::Mat img_gray = cv::Mat( img_text.size(), CV_8UC1)
          , img_gray2 = cv::Mat( img_text.size(), CV_8UC1)
          , img_gray_resize;
        int n_label = 0;
        const int MEAN = config.label_mean_rgb;
        for(int k=1;k>=0;k--){
          cv::cvtColor( *pi[k], img_gray2, cv::COLOR_BGR2GRAY);          
          cv::threshold(img_gray2, img_gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
          /*for(int i=img_gray.rows-1;i>=0;i--){
            unsigned char *p = img_gray.data + i*img_gray.step;
            for(int j=img_gray.cols-1;j>=0;j--){
              int v = config.label_multiply_rgb*(p[j]-MEAN) + MEAN;
              p[j] = v<0 ? 0 : (v>255 ? 255 : v);
            }
          }*/
          {
            // resize label
            const int label_h = 30
              , label_w = config.label_width * label_h / config.label_height;
            
            cv::resize(img_gray, img_gray_resize
              , cv::Size( label_w, label_h ));
          }
//          cv::resize(img_gray, img_gray_resize, cv::Size( 55, 30 ));
          ocr.get_text(img_gray_resize);

          for(int i=0;i<ocr.confs.size();i++){
            const char *str = ocr.texts[i].c_str();
            ROS_INFO("text flip [%d][%d]: %s, %.3f", ir, k, str, ocr.confs[i]);
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
                case 't':
                case 'T':
                  n_label = 1;
                  break;
                case 'a':
                case 'e':
                case '2':
                case '3':
                case 's':
                case 'S':
                  n_label = 2;
                  break;
                  /*
                case '3':
                case 's':
                case 'S':
                  n_label = 3;
                  break;*/
                default:
                  ROS_INFO("* Unknown M : %c\n", str2[0]);
              }
            }
          }
          if( config.show_result || config.save_result){
            char str[20];
            sprintf(str, "text_gray%d-%d.tif", ir, k);
            if( config.show_result )
              cv::imshow(str, img_gray_resize);
            if( config.save_result )
              cv::imwrite(config.result_save_path + str, img_gray_resize);
          }
          if( config.collect_mode ){
            char fname[256];
            create_filename( config.collect_prefix_label, fname, cnt_sample, n_label );
            cv::imwrite( fname, img_gray_resize );
            if( k==0 ){
              create_filename( config.collect_prefix_label_raw, fname, cnt_sample, n_label );
              cv::imwrite( fname, cv::Mat(img_col, cv::Range(r.y1, r.y2+1), cv::Range(r.x1, r.x2+1)) );
            }
            ROS_INFO("saved %d\n", cnt_sample);
            cnt_sample++;
          }


          if( n_label>0 ){
            cPlane plane(&r);
            if( !ransac_3d( r, cloud, plane ) ){
              ROS_INFO("* Label too few valid points [%d]\n", ir);
              continue;
            }
            plane.set_flip( k==1 );
            if( !plane.find_center_frames( warp_src) ){
              ROS_INFO("* find_center_frames failed [%d]\n", ir);
              continue;
            }
            plane.set_label( n_label );
            
            //  label size
            {
              double dis[2];
              for(int i=0;i<2;i++){
                dis[i] = sqrt( DIS2( plane.frames[i+1], plane.frames[i] ));
              }
              ROS_INFO("* label size 3D : %.3lf, %.3lf\n", dis[0], dis[1]);
              if( fabs(dis[0] - config.label_width) > config.label_size_error
                || fabs(dis[1] - config.label_height) > config.label_size_error){
                ROS_INFO("* invalid label size [%d]: %.3lf, %.3lf\n", ir, dis[0], dis[1]);
                continue;
              }
            }
            
            if( config.object_type==OBJECT_BOX ){
              plane.find_max_height(cloud, warp_src
                , config.pc_dis2plane, config.find_plane_radius);
            }
            else if( config.object_type==OBJECT_BASKET ){
              
            }
            ROS_INFO("** label : M%d\n", n_label);
            planes.push_back(plane);
            break;
          }
          else{
            ROS_INFO("* invalid label text [%d]\n", ir);
          }
        }
      }
      if( config.show_result || config.save_result ){
        char str[32];
        sprintf(str, "text%d.bmp", ir);
        if( config.show_result )
          cv::imshow(str, img_text);
        if( config.save_result )
          cv::imwrite(config.result_save_path + str, img_text);

        sprintf(str, "filter%d.bmp", ir);
        for(int i=0;i<4;i++){
          cv::line( img_result
          , warp_src[i]
          , warp_src[(i+1)%4]
          , cv::Scalar(200,200,200), 1);
        }
        if( config.show_result )
          cv::imshow(str, img_filter);
        if( config.save_result )
          cv::imwrite(config.result_save_path + str, img_filter);
        sprintf(str, "filter_edge%d.bmp", ir);
        if( config.show_result )
          cv::imshow(str, img_edge);
      }
    }
    if( config.show_result )
      cv::imshow( "result.bmp", img_result);
    if( config.save_result )
      cv::imwrite(config.result_save_path + "result.bmp", img_result);

//    printf("pick plane id : %d\n", pick_plane_id);
    if( config.object_type==OBJECT_BOX ){
      if( planes.size()>0 ){
        // sort by height
        std::vector<cPlane*> pp(planes.size());
        for(int i=0;i<planes.size();i++){
          pp[i] = &planes[i];
        }
        for(int i=0;i<planes.size()-1;i++){
          for(int j=i+1;j<planes.size();j++){
            if( pp[i]->max_height < pp[j]->max_height ){
              cPlane *p = pp[i];
              pp[i] = pp[j];
              pp[j] = p;
            }
          }
        }
        for(int i=0;i<planes.size();i++){
          pp[i]->order = i;
          if( i<planes.size()-1 )
            assert( pp[i]->max_height >= pp[i+1]->max_height );
        }

        // marker
        const double d = 1.0/255.0;
        geometry_msgs::Point p1, p2;
        for(int i=0;i<planes.size();i++){
          if( i>=pick_markers.size() )
            break;
          cPlane &plane = planes[i];//[pick_plane_id];
          plane.get_pick_pose();
          //plane.get_pick_pose( pick_pose );
          const int n_label = plane.label;
          const double arrow_size = plane.order==0 ? 0.1 : 0.03;
          visualization_msgs::Marker &marker = pick_markers[n_pick_marker]//[n_label-1]
            , &marker_frame = frame_markers[n_pick_marker++];
          
          marker.color.r = marker_frame.color.r = cols[n_label-1][0]/255.0;
          marker.color.g = marker_frame.color.g = cols[n_label-1][1]/255.0;
          marker.color.b = marker_frame.color.b = cols[n_label-1][2]/255.0;
          
          // arrow
          marker.action = marker.ADD;
          marker.points.clear();
          /*
          marker_point.x = plane.center.x - plane.normal.x * arrow_size;
          marker_point.y = plane.center.y - plane.normal.y * arrow_size;
          marker_point.z = plane.center.z - plane.normal.z * arrow_size;
          marker_point.x = plane.center.x;
          marker_point.y = plane.center.y;
          marker_point.z = plane.center.z;
        */
          plane.get_arrow_marker( p1, p2, arrow_size );
          marker.points.push_back(p2);
          marker.points.push_back(p1);

          // frame
          marker_frame.action = marker.ADD;
          marker_frame.points.clear();
          for(int i=0;i<5;i++){
            const pcl::PointXYZ &f = plane.frames[i%4];
            p1.x = f.x;
            p1.y = f.y;
            p1.z = f.z;
            marker_frame.points.push_back(p1);
          }
          ROS_INFO("label : %d, height : %.3lf\n", n_label, plane.max_height);
        }
      }
    }
    else if( config.object_type==OBJECT_BASKET ){
      const double x1 = config.box_label_shift_x
        , x2 = config.box_label_shift_x - config.box_size_x
        , y = config.box_size_y*0.5;
      const tf::Vector3 v_basket[4] = {
        tf::Vector3( x1, y, 0.0 ),
        tf::Vector3( x2, y, 0.0 ),
        tf::Vector3( x2,-y, 0.0 ),
        tf::Vector3( x1,-y, 0.0 )
      };
      place_markers.resize(planes.size());
      geometry_msgs::Point p1, p2;
      for(int i=0;i<planes.size();i++){
        cPlane &plane = planes[i];//[pick_plane_id];
        plane.get_place_pose();
        const int n_label = plane.label;
        const double arrow_size = 0.05;
        visualization_msgs::Marker &marker = place_markers[n_pick_marker]
          , &marker_frame = basket_markers[n_pick_marker++];
          
          
        marker.color.r = marker_frame.color.r = cols[n_label-1][0]/255.0;
        marker.color.g = marker_frame.color.g = cols[n_label-1][1]/255.0;
        marker.color.b = marker_frame.color.b = cols[n_label-1][2]/255.0;
        
        // arrow
        marker.action = marker.ADD;
        marker.points.clear();
        plane.get_arrow_marker( p1, p2, arrow_size );
        marker.points.push_back(p2);
        marker.points.push_back(p1);

        // frame
        marker_frame.action = marker.ADD;
        marker_frame.points.clear();
        geometry_msgs::Point &p = plane.pick_pose.pose.position;
        geometry_msgs::Quaternion &o = plane.pick_pose.pose.orientation;
        tf::Quaternion q(o.x, o.y, o.z, o.w);
        

        for(int i=0;i<5;i++){
          
          const tf::Vector3 v = tf::quatRotate( q, v_basket[i%4] );
          p1.x = v.x() + p.x;
          p1.y = v.y() + p.y;
          p1.z = v.z() + p.z;
          
          // draw label frame
          // have to change frame_id to FRAME_CAMERA
          /*
          marker_frame.header.frame_id = FRAME_CAMERA;
          const tf::Vector3 v( plane.frames[i%4].x, plane.frames[i%4].y, plane.frames[i%4].z );
          p1.x = v.x();
          p1.y = v.y();
          p1.z = v.z();
          */
          
          marker_frame.points.push_back(p1);
        }
      }
    }
  }


  void clear_markers(){
    for(int i=0;i<pick_markers.size();i++){
      pick_markers[i].action = visualization_msgs::Marker::DELETE;
      pick_markers[i].points.clear();

      frame_markers[i].action = visualization_msgs::Marker::DELETE;
      frame_markers[i].points.clear();

      place_markers[i].action = visualization_msgs::Marker::DELETE;
      place_markers[i].points.clear();

      basket_markers[i].action = visualization_msgs::Marker::DELETE;
      basket_markers[i].points.clear();
    }
/*    std::vector<visualization_msgs::Marker> markers(2);
    markers[0].ns = NS_PICK_ARROW;
    markers[1].ns = NS_LABEL_FRAME;
    markers[0].type = visualization_msgs::Marker::ARROW;
    markers[1].type = visualization_msgs::Marker::LINE_STRIP;
    for(int j=markers.size()-1;j>=0;j--){
      visualization_msgs::Marker &m = markers[j];
      m.header.frame_id = FRAME_CAMERA;
      m.scale.x = m.scale.y = m.scale.z = 0.005;
      for(int i=0;i<8;i++){
        m.id = i;
        pub.publish(m);
      }
    }*/
  }
};

#endif
