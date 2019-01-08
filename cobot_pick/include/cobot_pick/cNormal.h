
#ifndef __CNORMAL_H__
#define __CNORMAL_H__

#include "cobot_pick/common.h"
#include <opencv2/opencv.hpp>
#include <tbb/task_scheduler_init.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>


class cCalNormal3{
public:
	static int			ret;
private:
//	double		***map3d;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointXYZRGB *pnts;
  pcl::PointNormal *normals;
	int			WIN_SIZE;
	int			MIN_N;
	int			*valid_reg;

public:
	cCalNormal3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud
      , pcl::PointCloud<pcl::PointNormal>::Ptr &normal
      , int win_size):cloud(_cloud){
		//map3d = pf->map3d;
    pnts = &cloud->points[0];
		WIN_SIZE = win_size;
		MIN_N = WIN_SIZE/2;
    normals = &normal->points[0];
	}
	~cCalNormal3( void ){}
	
	void operator()( const tbb::blocked_range< int >& range ) const;

};

int cCalNormal3::ret;

void cCalNormal3::operator()( const tbb::blocked_range< int >& range ) const{
	int kx1, kx2, ky1, ky2;
	int n;
	unsigned char **ptr = new unsigned char*[WIN_SIZE*2+1];
	double nx, ny, nz, xx , xy, xz, yy, yz, zz;
//	double m[3][3], v[3];
	int rr[2];
  int valid_reg[] = { WIN_SIZE, WIN_SIZE
      , int(cloud->width)-WIN_SIZE-1, int(cloud->height)-WIN_SIZE-1};
	if( ptr==NULL ){
		printf("CalNormal3(): memory allocation failed.\n");
		ret = -1;
		return;
	}
	CvMat *var = cvCreateMat(3,3, CV_64F),
		*eig = cvCreateMat(3,1, CV_64F),
		*eig_v = cvCreateMat(3, 3, CV_64F);

	rr[0] = range.begin();
	rr[1] = range.end()-1;
	for(int i=rr[0];i<=rr[1];i++){
		//ky1 = ( i-WIN_SIZE<valid_reg[1] )? valid_reg[1] : i-WIN_SIZE;
		//ky2 = ( i+WIN_SIZE>valid_reg[3] )? valid_reg[3] : i+WIN_SIZE;
		ky1 = i-WIN_SIZE;
		ky2 = i+WIN_SIZE;
		
		//for(int l1=ky1;l1<=ky2;l1++)
		//	ptr[l1-ky1] = (unsigned char*)img_disp->imageData + img_disp->widthStep*l1;
		//unsigned char *ptr2 = (unsigned char*)img_disp->imageData + img_disp->widthStep*i;

		for(int j=valid_reg[0];j<=valid_reg[2];j++){
//			kx1 = ( j-WIN_SIZE<valid_reg[0] )? valid_reg[0] : j-WIN_SIZE;
//			kx2 = ( j+WIN_SIZE>valid_reg[2] )? valid_reg[2] : j+WIN_SIZE;
			kx1 = j-WIN_SIZE;
			kx2 = j+WIN_SIZE;
			////////////////////////
			if( j==valid_reg[0] ){
				n = 0;
				nx = ny = nz = xx = xy = xz = yy = yz = zz = 0.0;
				for(int l1=ky1;l1<=ky2;l1++){
          const int ly = l1*cloud->width;
					for(int l2=kx1;l2<=kx2;l2++){
						assert( l1>=0 && l1<cloud->height );
						assert( l2>=0 && l2<cloud->width );
            const register  pcl::PointXYZRGB &p = pnts[ly+l2]; //map3d[l1][l2];
						if( IS_VALID_POINT(p) ){
							nx+= p.x;
							ny+= p.y;
							nz+= p.z;
							xx+= p.x*p.x;
							xy+= p.x*p.y;
							xz+= p.x*p.z;
							yy+= p.y*p.y;
							yz+= p.y*p.z;
							zz+= p.z*p.z;
							n++;
						}
					}
				}
			}
			else{
				for(int l1=ky1;l1<=ky2;l1++){
          const int ly = l1*cloud->width;
          assert( l1>=0 && l1<cloud->height );
          const register pcl::PointXYZRGB &p1 = pnts[ly+kx1-1], &p2 = pnts[ly+kx2];
					if( IS_VALID_POINT(p1) ){
						nx-= p1.x;
            ny-= p1.y;
            nz-= p1.z;
            xx-= p1.x*p1.x;
            xy-= p1.x*p1.y;
            xz-= p1.x*p1.z;
            yy-= p1.y*p1.y;
            yz-= p1.y*p1.z;
            zz-= p1.z*p1.z;
						n--;
					}
          if( IS_VALID_POINT(p2) ){
					//if( ptr[l1-ky1][kx2]!=0 ){
						//register double *m = map3d[l1][kx2];
						nx+= p2.x;
            ny+= p2.y;
            nz+= p2.z;
            xx+= p2.x*p2.x;
            xy+= p2.x*p2.y;
            xz+= p2.x*p2.z;
            yy+= p2.y*p2.y;
            yz+= p2.y*p2.z;
            zz+= p2.z*p2.z;
						n++;
					}
				}
			}
			if( n<MIN_N || !IS_VALID_POINT(pnts[i*cloud->width + j]) ){
        pcl::PointNormal &np = normals[i*cloud->width + j];
				np.normal_x = np.normal_y = np.normal_z = INVALID_POINT;
				continue;
			}

			{	// PCA
				
				double mean[3];
				double dtmp;
				
				mean[0] = nx/n;
				mean[1] = ny/n;
				mean[2] = nz/n;
				dtmp = 1.0/(n-1);
				
				{
					double *v = var->data.db;
					v[0] = (xx - n*mean[0]*mean[0])*dtmp;
					v[4] = (yy - n*mean[1]*mean[1])*dtmp;
					v[8] = (zz - n*mean[2]*mean[2])*dtmp;
					v[1] = v[3] = (xy - n*mean[0]*mean[1])*dtmp;
					v[2] = v[6] = (xz - n*mean[0]*mean[2])*dtmp;
					v[5] = v[7] = (yz - n*mean[1]*mean[2])*dtmp;
				}
				cvEigenVV (var, eig_v, eig, 0.00000001);	//分散共分散行列の固有値の計算
			//	cvSVD(var, eig, eig_v, 0, CV_SVD_U_T + CV_SVD_MODIFY_A);
				assert( eig->data.db[0] >= eig->data.db[1] && eig->data.db[1] >= eig->data.db[2] );

        pcl::PointNormal &np = normals[i*cloud->width + j];
				if( eig_v->data.db[8]>0.0 ){
					np.normal_x = eig_v->data.db[6];
					np.normal_y = eig_v->data.db[7];
					np.normal_z = eig_v->data.db[8];
				}
				else{
					np.normal_x = -eig_v->data.db[6];
					np.normal_y = -eig_v->data.db[7];
					np.normal_z = -eig_v->data.db[8];
				}
			}
			
//			rad[i][j][0] = atan2( norm2[i][j][2], norm2[i][j][0] );
//			rad[i][j][1] = atan2( norm2[i][j][1], sqrt(POW2(norm2[i][j][0])+POW2(norm2[i][j][2])) );
		}
	}
	cvReleaseMat(&var);
	cvReleaseMat(&eig);
	cvReleaseMat(&eig_v);
	delete [] ptr;
}


int CalNormal3_with_TBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud
      , pcl::PointCloud<pcl::PointNormal>::Ptr &normal
      , int win_size, int thread_num){
  normal->width = cloud->width;
  normal->height = cloud->height;
  normal->points.resize(normal->width*normal->height);

	static cCalNormal3 cc(cloud, normal, win_size);
	cc.ret = 0;
//	tbb::parallel_for( tbb::blocked_range<int>( valid_reg[1], valid_reg[3]+1
//		, (valid_reg[3]+HAVE_TBB-valid_reg[1])/HAVE_TBB ), cc );
  tbb::parallel_for( tbb::blocked_range<int>( win_size
    , cloud->height-win_size
		, (cloud->height-win_size-1+thread_num-win_size)/thread_num ), cc );
	return cc.ret;
}


void add_normal_marker(std::vector<visualization_msgs::Marker> &markers
		, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud
		, int win_size, int i, int j, float *col
	){
	static int cnt = 0;
	double nx, ny, nz, xx , xy, xz, yy, yz, zz;
	int n = 0;
	nx = ny = nz = xx = xy = xz = yy = yz = zz = 0.0;

	geometry_msgs::Point p1;
	visualization_msgs::Marker marker;
	marker.header.frame_id = FRAME_CAMERA;
	marker.ns = "normal";
	marker.type = visualization_msgs::Marker::POINTS;
	marker.color.r = col[0];
	marker.color.g = col[1];
	marker.color.b = col[2];
	marker.color.a = 0.6;
	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.id = cnt++;
	for(int y=i-win_size;y<=i+win_size;y++){
		for(int x=j-win_size;x<=j+win_size;x++){
			pcl::PointXYZRGB &p = cloud->points[y*cloud->width+x];
			if( IS_VALID_POINT(p) ){
				n++;
				nx+= p.x;
				ny+= p.y;
				nz+= p.z;
				xx+= p.x*p.x;
				xy+= p.x*p.y;
				xz+= p.x*p.z;
				yy+= p.y*p.y;
				yz+= p.y*p.z;
				zz+= p.z*p.z;

				p1.x = p.x;
				p1.y = p.y;
				p1.z = p.z;
				marker.points.push_back(p1);
			}
		}
	}
	markers.push_back(marker);
	assert(n>3);

	CvMat *var = cvCreateMat(3,3, CV_64F),
		*eig = cvCreateMat(3,1, CV_64F),
		*eig_v = cvCreateMat(3, 3, CV_64F);
	double mean[3];
	double dtmp;
	mean[0] = nx/n;
	mean[1] = ny/n;
	mean[2] = nz/n;
	dtmp = 1.0/(n-1);
	
	double *v = var->data.db;
	v[0] = (xx - n*mean[0]*mean[0])*dtmp;
	v[4] = (yy - n*mean[1]*mean[1])*dtmp;
	v[8] = (zz - n*mean[2]*mean[2])*dtmp;
	v[1] = v[3] = (xy - n*mean[0]*mean[1])*dtmp;
	v[2] = v[6] = (xz - n*mean[0]*mean[2])*dtmp;
	v[5] = v[7] = (yz - n*mean[1]*mean[2])*dtmp;


	
	cvEigenVV (var, eig_v, eig, DBL_EPSILON);
	assert( eig->data.db[0] >= eig->data.db[1] && eig->data.db[1] >= eig->data.db[2] );
	pcl::PointNormal np;
	if( eig_v->data.db[8]>0.0 ){
		np.normal_x = eig_v->data.db[6];
		np.normal_y = eig_v->data.db[7];
		np.normal_z = eig_v->data.db[8];
	}
	else{
		np.normal_x = -eig_v->data.db[6];
		np.normal_y = -eig_v->data.db[7];
		np.normal_z = -eig_v->data.db[8];
	}


	geometry_msgs::Point p_mean;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.color.a = 1.0;
	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.id = cnt++;
	marker.points.clear();
	p_mean.x = mean[0];
	p_mean.y = mean[1];
	p_mean.z = mean[2];
	for(int i=0;i<3;i++){
		marker.points.push_back(p_mean);
		p1.x = p_mean.x + eig_v->data.db[i*3] * 0.01;
		p1.y = p_mean.y + eig_v->data.db[i*3+1] * 0.01;
		p1.z = p_mean.z + eig_v->data.db[i*3+2] * 0.01;
		marker.points.push_back(p1);
	}
	markers.push_back(marker);

	for(int i=0;i<3;i++){
		printf("v : %.10lf, %.10lf, %.10lf\n"
			, v[i*3], v[i*3+1], v[i*3+2]);
	}
	printf("eig : %lf, %lf, %lf\n", eig->data.db[0]
		, eig->data.db[1], eig->data.db[2] );
	for(int i=0;i<3;i++)
		printf("eig_v %d : %.3lf, %.3lf, %.3lf\n", i
			, eig_v->data.db[i*3], eig_v->data.db[i*3+1]
			, eig_v->data.db[i*3+2]);
	printf("n2 : %.3lf, %.3lf, %.3lf\n"
		, np.normal_x, np.normal_y, np.normal_z);
		
	cvReleaseMat(&var);
	cvReleaseMat(&eig);
	cvReleaseMat(&eig_v);
}

#endif