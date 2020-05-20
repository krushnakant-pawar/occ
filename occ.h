#ifndef OCC_H
#define OCC_H

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <QMutex>
using namespace cv;
using namespace std;
typedef struct decodedData{
    vector<uint8_t> msg;
    cv::Mat img;
}DECODED_DATA;

DECODED_DATA processFrame(cv::Mat);
#endif //OCC_H
