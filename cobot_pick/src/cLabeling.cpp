

#include <assert.h>
#include <opencv2/opencv.hpp>

/*
#ifdef _DEBUG
//Debug
#pragma comment(lib,"opencv_core245d.lib")
#pragma comment(lib,"opencv_imgproc245d.lib")
#pragma comment(lib,"opencv_highgui245d.lib")
#pragma comment(lib,"opencv_video245d.lib")
#else
//Release
#pragma comment(lib,"opencv_core245.lib")
#pragma comment(lib,"opencv_imgproc245.lib")
#pragma comment(lib,"opencv_highgui245.lib")
#pragma comment(lib,"opencv_video245.lib")
#endif
*/

#include "cobot_pick/cLabeling.h"


inline int Compress( const std::vector<int> &parents, int a){
	assert(a!=0);
	while( a!=parents[a] )
		a = parents[a];
	assert(a!=0);
	return a;
}

int cLabeling::Exec(IplImage *src, IplImage *des){
	
	if( src->width!=des->width || src->height!=des->height ){
		printf("cLabeling::Exec(): Invalid images size (src=(%d,%d),des=(%d,%d)).\n", src->width, src->height, des->width, des->height);
		return -1;
	}	
	if( src->nChannels!=3 || des->nChannels!=1 ){
		printf("cLabeling::Exec(): Invalid images channel (src=%d,des=%d).\n", src->nChannels, des->nChannels);
		return -1;
	}	
	if( src->depth!=8 || des->depth!=16 ){
		printf("cLabeling::Exec(): Invalid images depth (src=%d,des=%d).\n", src->depth, des->depth );
		return -1;
	}
	
	std::vector<int> parents;
	parents.reserve(512);
	
	const int w = src->width, h = src->height, step1 = src->widthStep, w2 = des->widthStep / 2;
	int index = 0;
	unsigned char *p1 = (unsigned char*)src->imageData;
	
	
	// first row
	{
		unsigned short *lb = (unsigned short*)des->imageData;
		lb[0] = 0;
		parents.push_back(index++);
		for(int j=1;j<w;j++){
			unsigned char *p = (unsigned char*)p1 + j*3;
			if( p[0]==p[-3] && p[1]==p[-2] && p[2]==p[-1] ){
				lb[j] = Compress( parents,lb[j-1] );
			}
			else{
				lb[j] = index;
				parents.push_back(index++);
			}
		}
	}
	for(int i=1;i<h;i++){
		unsigned short *lb = (unsigned short*)(des->imageData + des->widthStep*i);
		// first column
		{
			unsigned char *p = (unsigned char*)p1 + step1*i;
			if( p[0]==p[-step1] && p[1]==p[1-step1] && p[2]==p[2-step1] ){
				lb[0] = Compress( parents,lb[-w2] );
			}
			else{
				lb[0] = index;
				parents.push_back(index++);
			}
		}
		for(int j=1;j<w;j++){
			unsigned char *p = (unsigned char*)p1 + step1*i + j*3;
			if( p[0]==p[-step1] && p[1]==p[1-step1] && p[2]==p[2-step1] ){
				lb[j] = Compress( parents,lb[j-w2] );
				if( p[0]==p[-3] && p[1]==p[-2] && p[2]==p[-1] ){
					int n = Compress( parents,lb[j-1] );
					if( n!=lb[j] ){
						if( n<lb[j] ){
							lb[j] = parents[lb[j]] = n;
						}
						else
							parents[n] = lb[j];
					}
				}
			}
			else if( p[0]==p[-3] && p[1]==p[-2] && p[2]==p[-1] ){
				lb[j] = Compress( parents,lb[j-1] );
			}
			else{
				lb[j] = index;
				parents.push_back(index++);
			}
		}
	}	
	
	// relabel
	int reg_num = 0;
	for(int i=0;i<(int)parents.size();i++){
		if( i!=parents[i] ){
		//	assert( parents[parents[i]]==parents[parents[parents[i]]] );
			parents[i] = parents[parents[i]];
		}
		else{
			parents[i] = reg_num++;
		}
	}
	// init reg_info
	reg_info.resize(reg_num);
	for(int i=reg_num-1;i>=0;i--){
		tRegInfo &r = reg_info[i];
		r.n_label = i+1;
		r.pix_num = 0;
		r.x1 = r.y1 = INT_MAX;
		r.x2 = r.y2 = -INT_MAX;
	}
	// renew label number
	for(int i=h-1;i>=0;i--){
		unsigned short *lb = (unsigned short*)(des->imageData + des->widthStep*i);
		for(int j=w-1;j>=0;j--){
			lb[j] = parents[lb[j]];
			assert( lb[j]<reg_num );
			tRegInfo &r = reg_info[lb[j]-1];
			r.pix_num++;
			if( r.x1>j ) r.x1 = j;
			if( r.x2<j ) r.x2 = j;
			if( r.y1>i ) r.y1 = i;
			if( r.y2<i ) r.y2 = i;
		}
	}
	return 0;
}




int cLabeling::ExecBin(IplImage *src, IplImage *des){
	reg_info.clear();

	if( src->width!=des->width || src->height!=des->height ){
		printf("cLabeling::ExecBin(): Invalid images size (src=(%d,%d),des=(%d,%d)).\n", src->width, src->height, des->width, des->height);
		return -1;
	}	
	if( src->nChannels!=1 || des->nChannels!=1 ){
		printf("cLabeling::ExecBin(): Invalid images channel (src=%d,des=%d).\n", src->nChannels, des->nChannels);
		return -1;
	}	
	if( src->depth!=8 || des->depth!=16 ){
		printf("cLabeling::ExecBin(): Invalid images depth (src=%d,des=%d).\n", src->depth, des->depth );
		return -1;
	}
	
	std::vector<int> parents;
	parents.reserve(8192);
	
	const int w = src->width, h = src->height, step1 = src->widthStep, w2 = des->widthStep / 2;
	unsigned char *p1 = (unsigned char*)src->imageData;
	
	cvSetZero(des);
	// first row
	{
		unsigned short *lb = (unsigned short*)des->imageData;
		parents.push_back(0);
		if( p1[0]>0 ){
			lb[0] = 1;
			parents.push_back(1);
		}
		for(int j=1;j<w;j++){
			if( p1[j]>0 ){
				if( p1[j-1]>0 )
					lb[j] = Compress( parents,lb[j-1] );
				else{
					lb[j] = parents.size();
					parents.push_back(parents.size());
				}
			}
		}
	}
	for(int i=1;i<h;i++){
		unsigned short *lb = (unsigned short*)(des->imageData + des->widthStep*i);
		// first column
		{
			unsigned char *p = (unsigned char*)p1 + step1*i;
			if( p[0]>0 ){
				if( p[-step1]>0 )
					lb[0] = Compress( parents,lb[-w2] );
				else{
					lb[0] = parents.size();
					parents.push_back(parents.size());
				}
			}
		}
		for(int j=1;j<w;j++){
			unsigned char *p = (unsigned char*)p1 + step1*i + j;
			if( p[0]==0 )
				continue;
			if( p[-step1]>0 ){
				lb[j] = Compress( parents,lb[j-w2] );
				if( p[-1]>0 ){
					int n = Compress( parents,lb[j-1] );
					if( n!=lb[j] ){
						if( n<lb[j] ){
							lb[j] = parents[lb[j]] = n;
						}
						else
							parents[n] = lb[j];
					}
				}
			}
			else if( p[-1]>0 ){
				lb[j] = Compress( parents,lb[j-1] );
			}
			else{
				lb[j] = parents.size();
				parents.push_back(parents.size());
			}
		}
	}	
	
	// relabel
	int reg_num = 0;
	for(int i=1;i<(int)parents.size();i++){
		assert(parents[i]!=0);
		if( i!=parents[i] ){
		//	assert( parents[parents[i]]==parents[parents[parents[i]]] );
			parents[i] = parents[parents[i]];
		}
		else{
			parents[i] = ++reg_num;
		}
	}
	if( reg_num==0 ){
		return 0;
	}
	// init reg_info
	reg_info.resize(reg_num);
	
	for(int i=reg_num-1;i>=0;i--){
		tRegInfo &r = reg_info[i];
		r.n_label = i+1;
		r.pix_num = 0;
		r.x1 = r.y1 = INT_MAX;
		r.x2 = r.y2 = -INT_MAX;
	}
	// renew label number
	for(int i=h-1;i>=0;i--){
		unsigned short *lb = (unsigned short*)(des->imageData + des->widthStep*i);
		for(int j=w-1;j>=0;j--){
			if( lb[j]==0 )
				continue;
			lb[j] = parents[lb[j]];
			assert( lb[j]<=reg_num );
			tRegInfo &r = reg_info[lb[j]-1];
			r.pix_num++;
			if( r.x1>j ) r.x1 = j;
			if( r.x2<j ) r.x2 = j;
			if( r.y1>i ) r.y1 = i;
			if( r.y2<i ) r.y2 = i;
		}
	}
	return 0;
}



int cLabeling::CreateImageResult( IplImage *label, IplImage *result, bool b_reset ){
	const unsigned char col[][3] = {
			{250,0,0},{0,250,0},{0,0,250},
			{250,250,0},{0,250,250},{250,0,250},
			{150,250,0},{0,150,250},{250,0,150},
			{250,150,0},{0,250,150},{150,0,250} };
	const int ncol = sizeof(col)/3;

	if( label->width!=result->width || label->height!=result->height ){
		printf("cLabeling::CreateImageResult(): Invalid images size (src=(%d,%d),des=(%d,%d)).\n", label->width, label->height, result->width, result->height);
		return -1;
	}	
	if( label->nChannels!=1 || result->nChannels!=3 ){
		printf("cLabeling::CreateImageResult(): Invalid images channel (src=%d,des=%d).\n", label->nChannels, result->nChannels);
		return -1;
	}	
	if( label->depth!=16 || result->depth!=8 ){
		printf("cLabeling::CreateImageResult(): Invalid images depth (src=%d,des=%d).\n", label->depth, result->depth );
		return -1;
	}
	if( b_reset )
		memset( result->imageData, 0, result->widthStep * result->height );

	for(int i=label->height-1;i>=0;i--){
		unsigned short *lb = (unsigned short*)(label->imageData + label->widthStep*i);
		unsigned char *p = (unsigned char*)(result->imageData + result->widthStep*i);
		for(int j=label->width-1;j>=0;j--){
			if( lb[j]==0 )
				continue;
			unsigned char *pp = p + 3*j;

			register int n = (lb[j]-1)%ncol;
			pp[0] = col[n][0];
			pp[1] = col[n][1];
			pp[2] = col[n][2];
		}
	}
	return 0;
}



/////////////  8-neig  /////////////




int cLabeling::ExecBin8(IplImage *src, IplImage *des){
	reg_info.clear();

	if( src->width!=des->width || src->height!=des->height ){
		printf("cLabeling::ExecBin8(): Invalid images size (src=(%d,%d),des=(%d,%d)).\n", src->width, src->height, des->width, des->height);
		return -1;
	}	
	if( src->nChannels!=1 || des->nChannels!=1 ){
		printf("cLabeling::ExecBin8(): Invalid images channel (src=%d,des=%d).\n", src->nChannels, des->nChannels);
		return -1;
	}	
	if( src->depth!=8 || des->depth!=16 ){
		printf("cLabeling::ExecBin8(): Invalid images depth (src=%d,des=%d).\n", src->depth, des->depth );
		return -1;
	}
	
	std::vector<int> parents;
	parents.reserve(8192);
	
	const int w = src->width, h = src->height, step1 = src->widthStep, w2 = des->widthStep / 2;
	unsigned char *p1 = (unsigned char*)src->imageData;
	
	cvSetZero(des);
	// first row
	{
		unsigned short *lb = (unsigned short*)des->imageData;
		parents.push_back(0);
		if( p1[0]>0 ){
			lb[0] = 1;
			parents.push_back(1);
		}
		for(int j=1;j<w;j++){
			if( p1[j]>0 ){
				if( p1[j-1]>0 )
					lb[j] = Compress( parents,lb[j-1] );
				else{
					lb[j] = parents.size();
					parents.push_back(parents.size());
				}
				assert( lb[j]!=0 );
			}
		}
	}
	for(int i=1;i<h;i++){
		unsigned short *lb = (unsigned short*)(des->imageData + des->widthStep*i);
		// first column
		{
			unsigned char *p = (unsigned char*)p1 + step1*i;
			if( p[0]>0 ){
				uchar pp[2] = { p[-step1+1]!=0, p[-step1]!=0 };
				int x[2] = { -w2+1, -w2 };
				int nn[2];
				int n = INT_MAX;
				for(int i=1;i>=0;i--){
					if( pp[i]==0 )
						continue;
					nn[i] = Compress( parents, lb[x[i]] );
					if( nn[i]<n ){
						n = nn[i];
					}
				}
				if( n<INT_MAX ){
					lb[0] = n;
					for(int i=1;i>=0;i--){
						if( pp[i]!=0 ){
							parents[nn[i]] = n;
						}
					}
				}
				else{
					lb[0] = parents.size();
					parents.push_back(parents.size());
				}
				assert( lb[0]!=0 );
			}
		}
		for(int j=1;j<w-1;j++){
			unsigned char *p = (unsigned char*)p1 + step1*i + j;
			if( p[0]==0 )
				continue;

			{
				uchar pp[4] = { p[-step1+1]!=0, p[-step1]!=0, p[-step1-1]!=0, p[-1]!=0 };
				int x[4] = { j-w2+1, j-w2, j-w2-1, j-1 };
				int nn[4];
				int n = INT_MAX;
				for(int i=3;i>=0;i--){
					if( pp[i]==0 )
						continue;
					nn[i] = Compress( parents, lb[x[i]] );
					if( nn[i]<n ){
						n = nn[i];
					}
				}
				if( n<INT_MAX ){
					lb[j] = n;
					for(int i=3;i>=0;i--){
						if( pp[i]!=0 ){
							parents[nn[i]] = n;
						}
					}
				}
				else{
					lb[j] = parents.size();
					parents.push_back(parents.size());
				}
			}
		}

		// last column
		{
			unsigned char *p = (unsigned char*)p1 + step1*i + w - 1;
			if( p[0]>0 ){
				uchar pp[3] = { p[-step1]!=0, p[-step1-1]!=0, p[-1]!=0 };
				int x[3] = { -1, -2, w2-2 };
				int nn[3];
				int n = INT_MAX;
				for(int i=2;i>=0;i--){
					if( pp[i]==0 )
						continue;
					nn[i] = Compress( parents, lb[x[i]] );
					if( nn[i]<n ){
						n = nn[i];
					}
				}
				if( n<INT_MAX ){
					lb[w2-1] = n;
					for(int i=2;i>=0;i--){
						if( pp[i]!=0 ){
							parents[nn[i]] = n;
						}
					}
				}
				else{
					lb[w2-1] = parents.size();
					parents.push_back(parents.size());
				}
			}
		}
	}	
	
	// relabel
	int reg_num = 0;
	for(int i=1;i<(int)parents.size();i++){
		assert(parents[i]!=0);
		if( i!=parents[i] ){
		//	assert( parents[parents[i]]==parents[parents[parents[i]]] );
			parents[i] = parents[parents[i]];
		}
		else{
			parents[i] = ++reg_num;
		}
	}
	if( reg_num==0 ){
		return 0;
	}
	// init reg_info
	reg_info.resize(reg_num);
	
	for(int i=reg_num-1;i>=0;i--){
		tRegInfo &r = reg_info[i];
		r.n_label = i+1;
		r.pix_num = 0;
		r.x1 = r.y1 = INT_MAX;
		r.x2 = r.y2 = -INT_MAX;
	}
	// renew label number
	for(int i=h-1;i>=0;i--){
		unsigned short *lb = (unsigned short*)(des->imageData + des->widthStep*i);
		for(int j=w-1;j>=0;j--){
			if( lb[j]==0 )
				continue;
			lb[j] = parents[lb[j]];
			assert( lb[j]<=reg_num );
			tRegInfo &r = reg_info[lb[j]-1];
			r.pix_num++;
			if( r.x1>j ) r.x1 = j;
			if( r.x2<j ) r.x2 = j;
			if( r.y1>i ) r.y1 = i;
			if( r.y2<i ) r.y2 = i;
		}
	}
	return 0;
}
