#ifndef MYOCC_H
#define MYOCC_H
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include "opencv2/opencv.hpp"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <pthread.h>
#include <unistd.h>
#include <cmath>
using namespace cv;
using namespace std;

//#define bool int
#define HEIGHT 480 //max y value (rows)
#define WIDTH 640  //max x value (columns)
#define LOGSTART 'l'
#define LOGSTOP 't'
#define MIN_VAL(a, b) (a < b)? a : b

#define TRACKER "MOSSE"
#define MAX_THREADS 8
#define FPS 60

#define MAX_PAYLOAD 8
#define CONSEC_SAMPLES 12
//values inside lab
#define RED_THRESHOLD 40000
#define MIN_CUTOFF 750
//For longer distance
#define MAX_SUM_TO_SFD_RATIO 4


#define TOTAL_SFD_BITS 23

//#define LED_CUTOFF 253 //shadow
#define LED_CUTOFF 200 //sunlight

#define NUM_MESSAGES 8

uint8_t validMessages[NUM_MESSAGES] = {
    0b10000000,
    0b00011100,
    0b00010111,
    0b00001011,
    0b11101000,
    0b11011011,
    0b10110110,
    0b01100100
};

typedef struct ROIDATA{
    int32_t prev_max_sum;
    int32_t consec_sum[CONSEC_SAMPLES];
    int32_t consec_diff_avg[MAX_PAYLOAD];
    int frame_no;
    bool sfdDecodeFlag;
    int noOfBits;
    int sfdBitCount;
    int pseudo_frame_no;
    Mat background;
    int diff_avg_index;
}roiData;
#endif // MYOCC_H
