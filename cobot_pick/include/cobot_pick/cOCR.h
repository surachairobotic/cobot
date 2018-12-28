#ifndef __COCR_H__
#define __COCR_H__

#include "cobot_pick/common.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include "ros/ros.h"


class cOCR{
private:
  tesseract::TessBaseAPI *ocr;

public:
  std::vector<std::string> texts;
  std::vector<float> confs;

  cOCR(){
    ocr = new tesseract::TessBaseAPI();
    // Initialize tesseract to use English (eng) and the LSTM OCR engine.
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
    // Set Page segmentation mode to PSM_AUTO (3)
    ocr->SetPageSegMode(tesseract::PSM_SINGLE_WORD);
  }

  ~cOCR(){
    delete ocr;
  }

  void get_text(const cv::Mat &img){
    texts.clear();
    confs.clear();
    
    ocr->SetImage(img.data, img.cols, img.rows, img.channels(), img.step);
    ocr->Recognize(NULL);

    tesseract::ResultIterator *ri = ocr->GetIterator();
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
/*    else{
      printf("plane[%d] : no text found\n", i);
    }*/
  }
};

#endif