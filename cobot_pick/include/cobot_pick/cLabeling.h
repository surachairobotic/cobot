#ifndef __CLABELING_H__
#define __CLABELING_H__

#include <vector>
#include <opencv2/opencv.hpp>

struct tRegInfo{
	int n_label;
	int pix_num;
	int x1, y1, x2, y2;
};

class cLabeling{
public:
	std::vector<tRegInfo>	reg_info;

public:
	cLabeling(){}
	~cLabeling(){}
	
	int Exec(IplImage *src, IplImage *des);
	int ExecBin(IplImage *src, IplImage *des);
	int CreateImageResult( IplImage *label, IplImage *result, bool b_reset=true );

	inline int ExecBin(cv::Mat &src, cv::Mat &des){
		if( des.data==NULL || des.rows!=src.rows || src.cols!=des.cols || des.type()!=CV_16UC1 )
			des = cv::Mat( src.rows, src.cols, CV_16UC1 );
		IplImage is = src, id = des;
		return ExecBin( &is, &id );
	}
	inline int CreateImageResult( cv::Mat &label, cv::Mat &result, bool b_reset=true ){
		if( result.data==NULL || result.rows!=label.rows || label.cols!=result.cols || result.type()!=CV_8UC3 )
			result = cv::Mat( label.rows, label.cols, CV_8UC3 );
		IplImage is = label, id = result;
		return CreateImageResult( &is, &id, b_reset );
	}

	/////////////  8-neig  /////////////

	int ExecBin8(IplImage *src, IplImage *des);

	inline int ExecBin8(cv::Mat &src, cv::Mat &des){
		if( des.data==NULL || des.rows!=src.rows || src.cols!=des.cols || des.type()!=CV_16UC1 )
			des = cv::Mat( src.rows, src.cols, CV_16UC1 );
		IplImage is = src, id = des;
		return ExecBin8( &is, &id );
	}

};

#endif