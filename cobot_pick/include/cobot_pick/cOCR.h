#ifndef __COCR_H__
#define __COCR_H__

#include "cobot_pick/common.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include "ros/ros.h"
#include <vector>


class cOCR{
private:
  std::vector<tesseract::TessBaseAPI*> ocrs;

public:
  std::vector<std::string> texts;
  std::vector<float> confs;

  cOCR(){
    for(int i=0;i<2;i++){
      ocrs.push_back(new tesseract::TessBaseAPI());
      //ocrs[i]->SetVariable("tessedit_char_whitelist", "M123");
      // Initialize tesseract to use English (eng) and the LSTM OCR engine.
      ocrs[i]->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
      // Set Page segmentation mode to PSM_AUTO (3)
      ocrs[i]->SetPageSegMode( i==0 ? tesseract::PSM_SINGLE_BLOCK : tesseract::PSM_SINGLE_WORD);
    }
    
  }

  ~cOCR(){
    for(int i=ocrs.size()-1;i>=0;i--)
      delete ocrs[i];
  }

  void get_text(const cv::Mat &img){
    texts.clear();
    confs.clear();
    
    for(int i=ocrs.size()-1;i>=0;i--){
      ocrs[i]->SetImage(img.data, img.cols, img.rows, img.channels(), img.step);
      ocrs[i]->Recognize(NULL);
      tesseract::ResultIterator *ri = ocrs[i]->GetIterator();
      if (ri != 0){
        do{
          const char *symbol = ri->GetUTF8Text(tesseract::PageIteratorLevel::RIL_SYMBOL);
          if (symbol != 0){
            texts.push_back(std::string(symbol));
            confs.push_back(ri->Confidence(tesseract::PageIteratorLevel::RIL_SYMBOL));
            delete[] symbol;
          }
          else{
            //printf("plane[%d] : no symbol\n", i);
          }
        } while ((ri->Next(tesseract::PageIteratorLevel::RIL_SYMBOL)));
      }
    }
  }
};

#endif
