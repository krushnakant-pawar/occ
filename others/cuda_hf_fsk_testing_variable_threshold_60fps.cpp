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
#include "myMessages.h"
using namespace cv;
using namespace std;

//#define bool int
#define HEIGHT 480 //max y value (rows)
#define WIDTH 640  //max x value (columns)
#define LOGSTART 'l'
#define LOGSTOP 't'
#define MIN_VAL(a, b) (a < b)? a : b

#define TRACKER "MOSSE"
#define NUM_THREADS 4
#define FPS 60

#define MAX_PAYLOAD 8
#define CONSEC_SAMPLES 12
//values inside lab
#define RED_THRESHOLD 50000
#define MIN_CUTOFF 750
//For longer distance
#define MAX_SUM_TO_SFD_RATIO 4


#define TOTAL_SFD_BITS 23

//#define LED_CUTOFF 253 //shadow
#define LED_CUTOFF 200 //sunlight


struct thread_data {
	int tid;
	uint8_t *dataStart;
	int noOfElements;
};

static string input_stream = "14Mar/sunlight_movement_fsk_50k_hf_50_45hz_data_01100100.avi"; //inside lab
//static string input_stream = "sunlight_fsk_50k_hf_50_45hz_data_11101000.avi";
//static int input_stream = 2;

double prev_max_sum = 0;
double consec_six[CONSEC_SAMPLES];
double consec_diff_avg[MAX_PAYLOAD];

namespace occ {
uint32_t sum[NUM_THREADS];
}

vector<string> trackerTypes = { "BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW",
		"GOTURN", "MOSSE", "CSRT" };

// create tracker by name
Ptr<Tracker> createTrackerByName(string trackerType) {
	Ptr<Tracker> tracker;
	if (trackerType == trackerTypes[0])
		tracker = TrackerBoosting::create();
	else if (trackerType == trackerTypes[1])
		tracker = TrackerMIL::create();
	else if (trackerType == trackerTypes[2])
		tracker = TrackerKCF::create();
	else if (trackerType == trackerTypes[3])
		tracker = TrackerTLD::create();
	else if (trackerType == trackerTypes[4])
		tracker = TrackerMedianFlow::create();
	else if (trackerType == trackerTypes[5])
		tracker = TrackerGOTURN::create();
	else if (trackerType == trackerTypes[6])
		tracker = TrackerMOSSE::create();
	else if (trackerType == trackerTypes[7])
		tracker = TrackerKCF::create();
	else {
		cout << "Incorrect tracker name" << endl;
		cout << "Available trackers are: " << endl;
		for (vector<string>::iterator it = trackerTypes.begin();
				it != trackerTypes.end(); ++it)
			std::cout << " " << *it << endl;
	}
	return tracker;
}

void getRandomColors(vector<Scalar> &colors, int numColors) {
	RNG rng(0);
	for (int i = 0; i < numColors; i++)
		colors.push_back(
				Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
						rng.uniform(0, 255)));
}

bool checkIfRed(Mat value, int x, int y, int w, int h) {

	int i, j;
	int rows, cols;

	if (x < 0) {
		x = 0;
		w = w - x;
	}

	if (y < 0) {
		y = 0;
		h = h - y;
	}

	cols = MIN_VAL((x + w), WIDTH);
	rows = MIN_VAL((y + h), HEIGHT);
	uint threshold = 0;

	unsigned char *val = (unsigned char*) (value.data);
	bool flag = false;
	for (int i = y; i < rows; i++) {
		for (int j = x; j < cols; j++) {
			threshold += (uint) val[i * value.step + j];
			if (threshold > RED_THRESHOLD) {
				flag = true;
				break;
			}
		}
	}
	//cout << "threshold : " << threshold << endl;
	if (flag == true)
		return true;
	else
		return false;

}

void* addElements(void *data) {
	struct thread_data *tData = (thread_data*) data;

	uint8_t *arr;
	uint32_t s = 0;
	arr = tData->dataStart;
	uint8_t element;
	for (int i = 0; i < tData->noOfElements; i++) {
		//cout << arr[i] << "\t";
		element = arr[i];
		if (element > LED_CUTOFF)
			s += (uint32_t) arr[i];
	}
	occ::sum[tData->tid] = s;
	pthread_exit(NULL);
}

uint8_t findHammingDistance(uint8_t msg1, uint8_t msg2){
	uint8_t x = msg1 ^ msg2; 
    uint8_t setBits = 0; 
  
    while (x > 0) { 
        setBits += x & 1; 
        x >>= 1; 
    } 
  
    return setBits; 
}

uint8_t findClosestMsg(uint8_t rxMsg){
	uint8_t minHammingDistance = 8;
	uint8_t index = 0;
	for (int i = 0; i < NUM_MESSAGES; i++)
	{
		hammingDistances[i] = findHammingDistance(rxMsg, validMessages[i]);
		if (minHammingDistance > hammingDistances[i])
			{
				index = i;
				minHammingDistance = hammingDistances[i];
			}	
	}
	return validMessages[index];
	
}

bool comp(double a, double b) 
{ 
	return (a < b); 
} 

Rect getLedLocation(Mat leds){
	int edge_thresh = 100;
    Mat src_gray = leds;
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Rect ledRect;
    Canny( src_gray, canny_output, edge_thresh, edge_thresh*2, 3 );

    findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255,0,0);
       ledRect = boundingRect(contours[i]);
     }

   	return ledRect;
}
/*
uint8_t decodeSymbolCUDA(Mat img_cpu, int x, int y, int w, int h) {
	static int frame_no = 0; 
	cv::cuda::GpuMat img_gpu;
	img_gpu.upload(img_cpu);

	static int sfdDecodeFlag = 1;
	static uint8_t rxMsg = 0;
	int i, j;
	int rows, cols;

	if (x < 0) {
		x = 0;
		w = w - x; //check if it should be abs(x)
	}

	if (y < 0) {
		y = 0;
		h = h - y; //check if it should be abs(y)
	}

	cols = MIN_VAL((x + w), WIDTH);  //max column number for roi
	rows = MIN_VAL((y + h), HEIGHT); //max row number for roi

	cv::cuda::GpuMat hsv[3];
	cv::cuda::GpuMat hsvImg;
	cv::cuda::cvtColor(img_gpu, hsvImg, COLOR_BGR2HSV);

	cv::cuda::split(hsvImg, hsv);

	cv::cuda::GpuMat threshMat[3];
	cv::cuda::GpuMat threshMat_low[3];
	cv::cuda::GpuMat threshMat_high[3];
	cv::cuda::GpuMat mask1, mask2;

	cv::Scalar l_red_hsv_low(0, 230, LED_CUTOFF);
	cv::Scalar l_red_hsv_high(10, 255, 255);

	cv::Scalar u_red_hsv_low(170, 230, LED_CUTOFF);
	cv::Scalar u_red_hsv_high(180, 255, 255);

	if(sfdDecodeFlag == 1){
		l_red_hsv_low[2] = LED_CUTOFF;
	}
	else{
		l_red_hsv_low[2] = 100;
	}

	//For lower red space
	//for lower hue range
	cv::cuda::threshold(hsv[0], threshMat_low[0], l_red_hsv_low[0], 255 , cv::THRESH_BINARY);
	cv::cuda::threshold(hsv[0], threshMat_high[0], l_red_hsv_high[0], 255 , cv::THRESH_BINARY_INV);
	cv::cuda::bitwise_and(threshMat_low[0], threshMat_high[0], threshMat[0]);

	//for lower saturation range
	cv::cuda::threshold(hsv[1], threshMat_low[1], l_red_hsv_low[1], 255 , cv::THRESH_BINARY);
	cv::cuda::threshold(hsv[1], threshMat_high[1], l_red_hsv_high[1], 255 , cv::THRESH_BINARY_INV);
	cv::cuda::bitwise_and(threshMat_low[1], threshMat_high[1], threshMat[1]);

	//for lower value range
	cv::cuda::threshold(hsv[2], threshMat_low[2], l_red_hsv_low[2], 255 , cv::THRESH_BINARY);
	cv::cuda::threshold(hsv[2], threshMat_high[2], l_red_hsv_high[2], 255 , cv::THRESH_BINARY_INV);
	cv::cuda::bitwise_and(threshMat_low[2], threshMat_high[2], threshMat[2]);

	cv::cuda::GpuMat tmp1;
	
	cv::cuda::bitwise_and(threshMat[0], threshMat[1], tmp1);
	cv::cuda::bitwise_and(tmp1, threshMat[2], mask1);

	//For higher red space
	//for higher hue range
	cv::cuda::threshold(hsv[0], threshMat_low[0], u_red_hsv_low[0], 255 , cv::THRESH_BINARY);
	cv::cuda::threshold(hsv[0], threshMat_high[0], u_red_hsv_high[0], 255 , cv::THRESH_BINARY_INV);
	cv::cuda::bitwise_and(threshMat_low[0], threshMat_high[0], threshMat[0]);

	//for higher saturation range
	cv::cuda::threshold(hsv[1], threshMat_low[1], u_red_hsv_low[1], 255 , cv::THRESH_BINARY);
	cv::cuda::threshold(hsv[1], threshMat_high[1], u_red_hsv_high[1], 255 , cv::THRESH_BINARY_INV);
	cv::cuda::bitwise_and(threshMat_low[1], threshMat_high[1], threshMat[1]);

	//for higher value range
	cv::cuda::threshold(hsv[2], threshMat_low[2], u_red_hsv_low[2], 255 , cv::THRESH_BINARY);
	cv::cuda::threshold(hsv[2], threshMat_high[2], u_red_hsv_high[2], 255 , cv::THRESH_BINARY_INV);
	cv::cuda::bitwise_and(threshMat_low[2], threshMat_high[2], threshMat[2]);

	
	cv::cuda::bitwise_and(threshMat[0], threshMat[1], tmp1);
	cv::cuda::bitwise_and(tmp1, threshMat[2], mask2);
	

	cv::cuda::bitwise_or(mask1, mask2, mask1);
	
	//cv::imwrite("mask.jpg", maskImg);

	// uncomment to remove noise
	//blur(mask1, mask1, Size(3, 3));
	//imshow("Red Mask", mask1);
	cv::cuda::GpuMat result;

	cv::cuda::bitwise_and(hsv[2], hsv[2], result, mask1);
	
	//imshow("Red brightness", hsv[2]);

	double s = cv::cuda::sum(result)[0]; //sum in filtered led region

	//cout << "sum: "<< s << endl;
	Mat hsv_cpu;
	result.download(hsv_cpu);

	result.release();
	hsv[0].release();
	hsv[1].release();
	hsv[2].release();
	hsvImg.release();
	img_gpu.release();
	threshMat[0].release();
	threshMat[1].release();
	threshMat[2].release();
	threshMat_low[0].release();
	threshMat_low[1].release();
	threshMat_low[2].release();
	threshMat_high[0].release();
	threshMat_high[1].release();
	threshMat_high[2].release();
	tmp1.release();
	mask1.release();
	mask2.release();



	double curr_max_sum;
	
	curr_max_sum = (double) ((s > prev_max_sum) ? s : prev_max_sum);

	prev_max_sum = curr_max_sum;

	//search for first 1 of sfd
	int index;	
	static int noOfBits = 0;
	static int sfdBitCount = 0;
	static int pseudo_frame_no = 0;
	static int bkp_prev_max = 0;
	int sfdCutOff = curr_max_sum/MAX_SUM_TO_SFD_RATIO;

	static Mat background;
	Mat lamp;
	
	
	if((x + w) > WIDTH)
		w = WIDTH - x;
	if((y + h) > HEIGHT)
		h = HEIGHT - y;
	
	
	if(sfdDecodeFlag == 1)
	{
		
		if(s < sfdCutOff)
		{
			sfdBitCount++;
		}else
		{
			sfdBitCount = 0;
		}
		//cout <<"frameno: "  << frame_no << "\tcurr_max_sum: " << curr_max_sum << "\ts: "<< s <<"\tprev_max_sum: " << prev_max_sum << "\tsfdCutOff: "<< sfdCutOff << "\tsfdBitCount: "<<sfdBitCount<<endl;
		if(sfdBitCount == TOTAL_SFD_BITS)
		{
			sfdBitCount = 0;
			sfdDecodeFlag = 0;
			pseudo_frame_no = 0;
			//bkp_prev_max = curr_max_sum;
			// Setup a rectangle to define your region of interest
			Rect myROI(x, y, w, h);
			Mat croppedRef(hsv_cpu, myROI);
			croppedRef.copyTo(background);
		}
		
		frame_no++;
		return 0;
		
	}

	Mat leds;
	
	Rect myROI2(x, y, w, h);

	Mat croppedRef2(hsv_cpu, myROI2);
	croppedRef2.copyTo(lamp);

    resize(lamp, lamp, cvSize(background.size().width, background.size().height));
	
	absdiff(background, lamp, leds);

    Mat leds_original;
	leds.copyTo(leds_original);

	
    erode(leds, leds, Mat(), Point(-1, -1), 2, 1, 1); //default 3x3 kernel
    dilate(leds, leds, Mat(), Point(-1, -1), 2, 1, 1); //default 3x3 kernel

	//Mat element = getStructuringElement( MORPH_RECT, Size( 7, 7), Point(-1, -1));
	//erode(leds, leds, element); //custom kernel
    //dilate(leds, leds, element); //custom kernel

    //imshow("background",background);
    //imshow("lamp",lamp);
    
    //extract exact led location
    //Rect ledLocation = getLedLocation(leds);
    //rectangle(leds, ledLocation, Scalar(255,0,0),2);
    //imshow("leds",leds);
    //imshow("ledOriginal",leds_original);
    //Mat Exactleds;
    //Mat croppedRef3(leds_original, ledLocation);
    //croppedRef3.copyTo(Exactleds);

    //double led_s = cv::sum(Exactleds)[0]; //sum in original led region
    double led_s = cv::sum(leds)[0]; //sum in filtered led region


    index = pseudo_frame_no % 12;
    consec_six[index] = led_s;
    static string msg = "";
    static string dynamicMsg = "";
    static int diff_avg_index = 0;
    if(index == 11)
        {
            int64_t first_diff = abs(consec_six[0] - consec_six[6]);
            int64_t second_diff = abs(consec_six[1] - consec_six[7]);
            int64_t third_diff = abs(consec_six[2] - consec_six[8]);
            int64_t fourth_diff = abs(consec_six[3] - consec_six[9]);
            int64_t fifth_diff = abs(consec_six[4] - consec_six[10]);
            int64_t sixth_diff = abs(consec_six[5] - consec_six[11]);
                
            int64_t diff_avg = (first_diff + second_diff + third_diff + fourth_diff + fifth_diff + sixth_diff)/3; //keep it as 3
            consec_diff_avg[diff_avg_index] = diff_avg;    
            diff_avg_index++;

            int count_diff = 0;
                
            if(first_diff > MIN_CUTOFF)
                count_diff++;
            if(second_diff > MIN_CUTOFF)
                count_diff++;
            if(third_diff > MIN_CUTOFF)
                count_diff++;
            if(fourth_diff > MIN_CUTOFF)
                count_diff++;
            if(fifth_diff > MIN_CUTOFF)
                count_diff++;
            if(sixth_diff > MIN_CUTOFF)
                count_diff++;
            int current_bit = (count_diff>3) ? 1 : 0;
	        //cout << setw(10) << (frame_no)<< setw(10) << first_diff << setw(10) << second_diff  << setw(10) << third_diff << setw(10) << fourth_diff << setw(10) << fifth_diff << setw(10) << sixth_diff << setw(10) << diff_avg << setw(10) << current_bit << endl;
            
	        if (current_bit != 0)
	        {
	        	msg += "1";
	        }
	        else
	        {
	        	msg += "0";
	        }
            noOfBits++;
			if(noOfBits == MAX_PAYLOAD){
				sfdDecodeFlag = 1;
				sfdBitCount = 0;
				//cout << "!!!!!!!!! Rx message 1: " << msg << endl;
				msg = "";
				noOfBits = 0;
				pseudo_frame_no = 0;
				//prev_max_sum = bkp_prev_max;
				prev_max_sum = 0;

				diff_avg_index = 0;

				double* max_diff_avg = std::max_element(consec_diff_avg, consec_diff_avg + MAX_PAYLOAD, comp);
				double* min_diff_avg = std::min_element(consec_diff_avg, consec_diff_avg + MAX_PAYLOAD, comp);

				int64_t dynamicCutOff = 0;
				for (int k = 0; k < MAX_PAYLOAD; k++)
				{
					dynamicCutOff += consec_diff_avg[k]; 
				}
				dynamicCutOff = dynamicCutOff/MAX_PAYLOAD;


				for(int k = 0; k < MAX_PAYLOAD; k++){
					if (consec_diff_avg[k] > dynamicCutOff)
					{
						//dynamicMsg += "1";
						rxMsg = rxMsg << 1;
						rxMsg |= 0x1;
					}else
					{
						//dynamicMsg += "0";
						rxMsg = rxMsg << 1;
					}
				}
				cout << setw(50) << "" << "Received Msg:  " << setw(12) << std::bitset<8>(rxMsg) << endl;

				rxMsg = findClosestMsg(rxMsg);

				
				cout << setw(50) << "" << "Corrected Msg: " << setw(12) << std::bitset<8>(rxMsg) << endl << endl;
				rxMsg = 0;
				//dynamicMsg = "";
			}			
              
        }
	pseudo_frame_no++;
    frame_no++;
	
	
	return 0;


}
*/
uint8_t decodeSymbolFromCroppedImg(Mat img, int x, int y, int w, int h) {
	static int frame_no = 0; 

	static int sfdDecodeFlag = 1;
	static uint8_t rxMsg = 0;
	int i, j;


	if (x < 0) {
		x = 0;
		w = w - x; //check if it should be abs(x)
	}

	if (y < 0) {
		y = 0;
		h = h - y; //check if it should be abs(y)
	}

	Mat hsv[3];
	Mat hsvImg;
	cvtColor(img, hsvImg, CV_BGR2HSV);

	Mat mask1, mask2;

	if(sfdDecodeFlag == 1){
		inRange(hsvImg, Scalar(0, 230, LED_CUTOFF), Scalar(10, 255, 255), mask1);
		inRange(hsvImg, Scalar(170, 230, LED_CUTOFF), Scalar(180, 255, 255), mask2);

	}
	else{
		inRange(hsvImg, Scalar(0, 230, 100), Scalar(10, 255, 255), mask1);
	    inRange(hsvImg, Scalar(170, 230, 100), Scalar(180, 255, 255), mask2);
	}

	mask1 = mask1 | mask2;

	// uncomment to remove noise
	//blur(mask1, mask1, Size(3, 3));
	//imshow("Red Mask", mask1);
	
	Mat result;

	bitwise_and(hsvImg, hsvImg, result, mask1);
	split(result, hsv);
	//imshow("Red brightness", hsv[2]);

	double s = cv::sum(hsv[2])[0]; //sum in filtered led region

	double curr_max_sum;
	
	curr_max_sum = (double) ((s > prev_max_sum) ? s : prev_max_sum);

	prev_max_sum = curr_max_sum;

	//search for first 1 of sfd
	int index;	
	static int noOfBits = 0;
	static int sfdBitCount = 0;
	static int pseudo_frame_no = 0;
	static int bkp_prev_max = 0;
	int sfdCutOff = curr_max_sum/MAX_SUM_TO_SFD_RATIO;

	static Mat background;
	Mat lamp;
	
	/*
	if((x + w) > WIDTH)
		w = WIDTH - x;
	if((y + h) > HEIGHT)
		h = HEIGHT - y;
	*/
	
	if(sfdDecodeFlag == 1)
	{
		
		if(s < sfdCutOff)
		{
			sfdBitCount++;
		}else
		{
			sfdBitCount = 0;
		}
		//cout <<"frameno: "  << frame_no << "\tcurr_max_sum: " << curr_max_sum << "\ts: "<< s <<"\tprev_max_sum: " << prev_max_sum << "\tsfdCutOff: "<< sfdCutOff << "\tsfdBitCount: "<<sfdBitCount<<endl;
		if(sfdBitCount == TOTAL_SFD_BITS)
		{
			sfdBitCount = 0;
			sfdDecodeFlag = 0;
			pseudo_frame_no = 0;
			//bkp_prev_max = curr_max_sum;
			// Setup a rectangle to define your region of interest
			//Rect myROI(x, y, w, h);
			//Mat croppedRef(hsv[2], myROI);
			hsv[2].copyTo(background);
		}
		
		frame_no++;
		return 0;
		
	}

	Mat leds;
	
	//Rect myROI2(x, y, w, h);

	//Mat croppedRef2(hsv[2], myROI2);
	hsv[2].copyTo(lamp);

    resize(lamp, lamp, cvSize(background.size().width, background.size().height));
	
	absdiff(background, lamp, leds);

    Mat leds_original;
	leds.copyTo(leds_original);
	
    erode(leds, leds, Mat(), Point(-1, -1), 2, 1, 1); //default 3x3 kernel
    dilate(leds, leds, Mat(), Point(-1, -1), 2, 1, 1); //default 3x3 kernel

	//Mat element = getStructuringElement( MORPH_RECT, Size( 7, 7), Point(-1, -1));
	//erode(leds, leds, element); //custom kernel
    //dilate(leds, leds, element); //custom kernel

    imshow("background",background);
    //imshow("lamp",lamp);
    
    //extract exact led location
    Rect ledLocation = getLedLocation(leds);
    rectangle(leds, ledLocation, Scalar(255,0,0),2);
    imshow("leds",leds);
    //imshow("ledOriginal",leds_original);
    Mat Exactleds;
    //Mat croppedRef3(leds_original, ledLocation);
    //croppedRef3.copyTo(Exactleds);

    //double led_s = cv::sum(Exactleds)[0]; //sum in original led region
    double led_s = cv::sum(leds)[0]; //sum in filtered led region


    index = pseudo_frame_no % 12;
    consec_six[index] = led_s;
    static string msg = "";
    static string dynamicMsg = "";
    static int diff_avg_index = 0;
    if(index == 11)
        {
            int64_t first_diff = abs(consec_six[0] - consec_six[6]);
            int64_t second_diff = abs(consec_six[1] - consec_six[7]);
            int64_t third_diff = abs(consec_six[2] - consec_six[8]);
            int64_t fourth_diff = abs(consec_six[3] - consec_six[9]);
            int64_t fifth_diff = abs(consec_six[4] - consec_six[10]);
            int64_t sixth_diff = abs(consec_six[5] - consec_six[11]);
                
            int64_t diff_avg = (first_diff + second_diff + third_diff + fourth_diff + fifth_diff + sixth_diff)/3; //keep it as 3
            consec_diff_avg[diff_avg_index] = diff_avg;    
            diff_avg_index++;

            int count_diff = 0;
                
            if(first_diff > MIN_CUTOFF)
                count_diff++;
            if(second_diff > MIN_CUTOFF)
                count_diff++;
            if(third_diff > MIN_CUTOFF)
                count_diff++;
            if(fourth_diff > MIN_CUTOFF)
                count_diff++;
            if(fifth_diff > MIN_CUTOFF)
                count_diff++;
            if(sixth_diff > MIN_CUTOFF)
                count_diff++;
            int current_bit = (count_diff>3) ? 1 : 0;
	        //cout << setw(10) << (frame_no)<< setw(10) << first_diff << setw(10) << second_diff  << setw(10) << third_diff << setw(10) << fourth_diff << setw(10) << fifth_diff << setw(10) << sixth_diff << setw(10) << diff_avg << setw(10) << current_bit << endl;
            
	        if (current_bit != 0)
	        {
	        	msg += "1";
	        }
	        else
	        {
	        	msg += "0";
	        }
            noOfBits++;
			if(noOfBits == MAX_PAYLOAD){
				sfdDecodeFlag = 1;
				sfdBitCount = 0;
				//cout << "!!!!!!!!! Rx message 1: " << msg << endl;
				msg = "";
				noOfBits = 0;
				pseudo_frame_no = 0;
				//prev_max_sum = bkp_prev_max;
				prev_max_sum = 0;

				diff_avg_index = 0;

				double* max_diff_avg = std::max_element(consec_diff_avg, consec_diff_avg + MAX_PAYLOAD, comp);
				double* min_diff_avg = std::min_element(consec_diff_avg, consec_diff_avg + MAX_PAYLOAD, comp);

				int64_t dynamicCutOff = 0;
				for (int k = 0; k < MAX_PAYLOAD; k++)
				{
					dynamicCutOff += consec_diff_avg[k]; 
				}
				dynamicCutOff = dynamicCutOff/MAX_PAYLOAD;


				for(int k = 0; k < MAX_PAYLOAD; k++){
					if (consec_diff_avg[k] > dynamicCutOff)
					{
						//dynamicMsg += "1";
						rxMsg = rxMsg << 1;
						rxMsg |= 0x1;
					}else
					{
						//dynamicMsg += "0";
						rxMsg = rxMsg << 1;
					}
				}
				cout << setw(50) << "" << "Received Msg:  " << setw(12) << std::bitset<8>(rxMsg) << endl;

				rxMsg = findClosestMsg(rxMsg);

				
				cout << setw(50) << "" << "Corrected Msg: " << setw(12) << std::bitset<8>(rxMsg) << endl << endl;
				rxMsg = 0;
				//dynamicMsg = "";
			}			
              
        }
	pseudo_frame_no++;
    frame_no++;
	
	
	return 0;


}

int processFrame() {
	
	VideoCapture cap(input_stream); //capture the video from web cam
	cout << "Input Stream:\t" << input_stream << endl;
	char ch;

	if (!cap.isOpened()) {
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	cap.set(CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, HEIGHT);
	cap.set(CV_CAP_PROP_FPS, FPS);
	cap.set(CAP_PROP_AUTO_EXPOSURE, 0.75); //For automatic exposure

	Rect box;

	cv::CascadeClassifier classifier("car-tail-lamp-cascade-20stages.xml");

	cv::Mat frame;
	cv::Mat grayscale_image;

	std::vector<cv::Rect> features;

	for (int i = 0; i < 30; ++i)
	{
		cap.read(frame);
	}

	while (true) {
		while (true) {
			cap.read(frame);
			prev_max_sum = 0.0;
			//detection loop:
			cv::cvtColor(frame, grayscale_image, COLOR_BGR2GRAY);
			//cv::equalizeHist(grayscale_image, grayscale_image);

			classifier.detectMultiScale(grayscale_image, features, 4, 4, 0 | 1,
					cv::Size(30, 30));

			int x = 0, y = 0, h = 0, w = 0;

			Mat hsv[3];
			Mat hsvImg;
			cvtColor(frame, hsvImg, CV_BGR2HSV);
			split(hsvImg, hsv);
			Mat mask1, mask2;

			inRange(hsvImg, Scalar(0, 230, 100), Scalar(10, 255, 255), mask1);
			inRange(hsvImg, Scalar(170, 230, 100), Scalar(180, 255, 255),
					mask2);

			mask1 = mask1 | mask2;

			// uncomment to remove noise
			blur(mask1, mask1, Size(3, 3));

			Mat result;
			bitwise_and(hsvImg, hsvImg, result, mask1);

			split(result, hsv);
			for (auto &&feature : features) {

				if (checkIfRed(hsv[2], feature.x, feature.y, feature.width,
						feature.height) == true) {
					if (h < feature.height) {
						x = feature.x;
						y = feature.y;
						h = feature.height;
						w = feature.width;
						//break;
					}

				}
			}

			//move this inside for loop to detect multiple objects
			if (h > 40) {
				box = cv::Rect(x, y, w, h);
				cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
				break;
			}

			//imshow("MultiTracker", frame);
			if (waitKey(1) == 27) {
				ch = 'q';
				break;
			}
		}

		if (ch == 'q') {
			ch = 't';
			break;
		}

		//tracking logic:
		string trackerType = TRACKER;

		Ptr<cv::MultiTracker> multiTracker = cv::MultiTracker::create();
		multiTracker->add(createTrackerByName(trackerType), frame, Rect2d(box));

		//tracking loop:
		int64 prev = 0;
		int imno = 0;
		double freq = cv::getTickFrequency();
		while (cap.isOpened()) {

			//Measure initial time ticks

                	
			int64 work_begin = cv::getTickCount();
			double time_elapsed = (work_begin - prev) / freq;
			//cout << "freq: " << time_elapsed << endl;

			if (time_elapsed > 1. / FPS)
			{
				prev = cv::getTickCount();
				//cout << "freq: " << 1. / time_elapsed << endl;
				cap.read(frame);
			
				// stop the program if reached end of video
				if (frame.empty())
					break;

				//update the tracking result with new frame
				multiTracker->update(frame);
				
				// draw tracked objects
				for (unsigned i = 0; i < multiTracker->getObjects().size();
						i++) {
					rectangle(frame, multiTracker->getObjects()[i],cv::Scalar(0, 255, 0), 2);
					
					Mat croppedRef(frame, multiTracker->getObjects()[i]);
					decodeSymbolFromCroppedImg(croppedRef, multiTracker->getObjects()[i].x,
					//decodeSymbolCUDA(frame, multiTracker->getObjects()[i].x,
							multiTracker->getObjects()[i].y,
							multiTracker->getObjects()[i].width,
							multiTracker->getObjects()[i].height);

				}

				imshow("MultiTracker", frame);
				
				// quit on ESC button
				if (waitKey(1) == 27)
					break;
				

			}

		}

	}
	
	cap.release();
	return 0;
}

