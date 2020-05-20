#include "occ.h"
#include "myocc.h"

bool detect = true;
QMutex detectMutex;

cv::CascadeClassifier classifier("/tmp/car-tail-lamp-cascade-20stages.xml");

string trackerType = TRACKER;
Ptr<cv::MultiTracker> multiTracker;

std::vector<roiData> sources;

//int32_t prev_max_sum = 0;
//int32_t consec_sum[CONSEC_SAMPLES];
//int32_t consec_diff_avg[MAX_PAYLOAD];

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

    uint8_t hammingDistances[NUM_MESSAGES];
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

bool comp(int32_t a, int32_t b)
{
    return (a < b);
}

vector<Rect> getOuterContours(Mat grayImg){
    int edge_thresh = 100;
    Mat src_gray = grayImg;
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Rect> contRect;
    Canny( src_gray, canny_output, edge_thresh, edge_thresh*2, 3 );

    findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for(unsigned int i = 0; i < contours.size(); i++ )
    {
        contRect.push_back(boundingRect(contours[i]));
    }

    return contRect;
}

uint8_t decodeSymbolFromCroppedImg(Mat img, roiData *roi) {
    //static int frame_no = 0;

    //static bool sfdDecodeFlag = true;
    uint8_t rxMsg = 0;

    Mat hsv[3];
    Mat hsvImg;
    cvtColor(img, hsvImg, CV_BGR2HSV);

    Mat mask1, mask2;

    if(roi->sfdDecodeFlag){
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

    int32_t s = cv::sum(hsv[2])[0]; //sum in filtered led region

    int32_t curr_max_sum;

    curr_max_sum = ((s > roi->prev_max_sum) ? s : roi->prev_max_sum);

    roi->prev_max_sum = curr_max_sum;

    //search for first 1 of sfd
    int index;
    //static int noOfBits = 0;
    //static int sfdBitCount = 0;
    //static int pseudo_frame_no = 0;
    int sfdCutOff = curr_max_sum/MAX_SUM_TO_SFD_RATIO;

    //static Mat background;
    Mat lamp;

    /*
    if((x + w) > WIDTH)
        w = WIDTH - x;
    if((y + h) > HEIGHT)
        h = HEIGHT - y;
    */

    if(roi->sfdDecodeFlag)
    {

        if(s < sfdCutOff)
        {
            roi->sfdBitCount++;
        }else
        {
            roi->sfdBitCount = 0;
        }
        if(roi->sfdBitCount == TOTAL_SFD_BITS)
        {
            roi->sfdBitCount = 0;
            roi->sfdDecodeFlag = false;
            roi->pseudo_frame_no = 0;
            //bkp_prev_max = curr_max_sum;
            // Setup a rectangle to define your region of interest
            //Rect myROI(x, y, w, h);
            //Mat croppedRef(hsv[2], myROI);
            hsv[2].copyTo(roi->background);
        }

        roi->frame_no++;
        return 0;

    }

    Mat leds;

    //Rect myROI2(x, y, w, h);

    //Mat croppedRef2(hsv[2], myROI2);
    hsv[2].copyTo(lamp);

    resize(lamp, lamp, cvSize(roi->background.size().width, roi->background.size().height));

    absdiff(roi->background, lamp, leds);

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
    vector<Rect>ledLocation = getOuterContours(leds);
    if(ledLocation.size() > 0){
        rectangle(leds, ledLocation[0], Scalar(255,0,0),2);
    }

    //imshow("leds",leds);
    //imshow("ledOriginal",leds_original);
    Mat Exactleds;
    //Mat croppedRef3(leds_original, ledLocation);
    //croppedRef3.copyTo(Exactleds);

    //double led_s = cv::sum(Exactleds)[0]; //sum in original led region
    int32_t led_s = cv::sum(leds)[0]; //sum in filtered led region


    index = roi->pseudo_frame_no % 12;
    roi->consec_sum[index] = led_s;

    //static int diff_avg_index = 0;
    if(index == 11)
    {
        int64_t first_diff = abs(roi->consec_sum[0] - roi->consec_sum[6]);
        int64_t second_diff = abs(roi->consec_sum[1] - roi->consec_sum[7]);
        int64_t third_diff = abs(roi->consec_sum[2] - roi->consec_sum[8]);
        int64_t fourth_diff = abs(roi->consec_sum[3] - roi->consec_sum[9]);
        int64_t fifth_diff = abs(roi->consec_sum[4] - roi->consec_sum[10]);
        int64_t sixth_diff = abs(roi->consec_sum[5] - roi->consec_sum[11]);

        int64_t diff_avg = (first_diff + second_diff + third_diff + fourth_diff + fifth_diff + sixth_diff)/3; //keep it as 3
        roi->consec_diff_avg[roi->diff_avg_index] = diff_avg;
        roi->diff_avg_index++;

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

        roi->noOfBits++;
        if(roi->noOfBits == MAX_PAYLOAD){
            roi->sfdDecodeFlag = true;
            roi->sfdBitCount = 0;
            roi->noOfBits = 0;
            roi->pseudo_frame_no = 0;
            //prev_max_sum = bkp_prev_max;
            roi->prev_max_sum = 0;

            roi->diff_avg_index = 0;
            int64_t dynamicCutOff = 0;
            for (int k = 0; k < MAX_PAYLOAD; k++)
            {
                dynamicCutOff += roi->consec_diff_avg[k];
            }
            dynamicCutOff = dynamicCutOff/MAX_PAYLOAD;


            for(int k = 0; k < MAX_PAYLOAD; k++){
                if (roi->consec_diff_avg[k] > dynamicCutOff)
                {
                    rxMsg = rxMsg << 1;
                    rxMsg |= 0x1;
                }else
                {
                    rxMsg = rxMsg << 1;
                }
            }

            rxMsg = findClosestMsg(rxMsg);
            //cout << "Corrected Msg: " << setw(12) << std::bitset<8>(rxMsg) << endl << endl;

        }

    }
    roi->pseudo_frame_no++;
    roi->frame_no++;


    return rxMsg;


}

DECODED_DATA processFrame(Mat frame) {
    DECODED_DATA decoded_data;
    decoded_data.msg.push_back(0);
    decoded_data.img = frame;
    //Mat img = cv::imread(":/img/temp.jpg");
    //cv::imshow("tmp", img);
    static int dummy = 0;
    if(dummy < 30)
    {
        dummy ++;
        return decoded_data;
    }

    Rect box;
    cv::Mat grayscale_image;
    if(frame.empty())
    {
        cout << "EMPTY FRAME" << endl;
        return decoded_data;
    }
    std::vector<cv::Rect> features;

    if (detect)
    {

        //prev_max_sum = 0; // ********************* check if required, removed after roiData
        //detection loop:
        cv::cvtColor(frame, grayscale_image, COLOR_BGR2GRAY);
        //cv::equalizeHist(grayscale_image, grayscale_image);
        if(classifier.empty()){
            cout << "Classifier not loaded" << endl;
            return decoded_data;
        }
        classifier.detectMultiScale(grayscale_image, features, 1.5, 4, 0 | 1,
                                    cv::Size(10, 10));

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
        cout << "Rect found: " << features.size() << endl;
        multiTracker = cv::MultiTracker::create();
        vector<Rect> bboxes;
        Mat blank = Mat::zeros(frame.rows, frame.cols,CV_8UC1);
        for (auto &&feature : features) {

            if (checkIfRed(hsv[2], feature.x, feature.y, feature.width,
                           feature.height) == true)
            {

                x = feature.x;
                y = feature.y;
                h = feature.height;
                w = feature.width;
                //break;
                if(h>40){
                    bboxes.push_back(Rect2d(cv::Rect(x,y,w,h)));
                    cv::rectangle(blank, Rect2d(cv::Rect(x,y,w,h)), cv::Scalar(255, 255, 255), 2);
                }

            }
        }

        bboxes = getOuterContours(blank);
        cout << "Detection complete: " << bboxes.size() << endl;

        for(int i = 0;i < bboxes.size(); i++){
            box = bboxes[i];
            detectMutex.lock();
            detect = false;
            detectMutex.unlock();
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
            multiTracker->add(createTrackerByName(trackerType), frame, Rect2d(box));

            roiData roi;
            roi.frame_no = 0;
            roi.noOfBits = 0;
            roi.sfdBitCount = 0;
            roi.prev_max_sum = 0;
            roi.diff_avg_index = 0;
            roi.pseudo_frame_no = 0;
            roi.sfdDecodeFlag = true;

            sources.push_back(roi);
        }
        //cv::imshow("no name", blank);
        //cv::waitKey(20);
    }else
    {
        //tracking logic:
        int64_t start = cv::getTickCount();


        multiTracker->update(frame);
        // draw tracked objects
        decoded_data.msg.clear();
        for (unsigned i = 0; i < multiTracker->getObjects().size();i++)
        {
            box = multiTracker->getObjects()[i];
            rectangle(frame, box,cv::Scalar(0, 255, 0), 2);
            putText(frame, "ID: " + to_string(i), Point(box.x, box.y), FONT_HERSHEY_PLAIN,2, cv::Scalar(255, 0, 0),2);
            box.x = (box.x > 0 ? box.x : 0);
            box.y = (box.y > 0 ? box.y : 0);
            box.width = abs((box.x > 0) ? box.width : box.width - abs(box.x));
            box.height = abs((box.y > 0)? box.height : box.height - abs(box.y));

            Mat croppedRef(frame, box);
            uint8_t decMsg = decodeSymbolFromCroppedImg(croppedRef,&sources[i]);
            decoded_data.msg.push_back(decMsg);
        }

        int64_t stop = cv::getTickCount();
        cout << "FPS: " << std::trunc(cv::getTickFrequency()/(stop-start)) << endl;

    }
    return decoded_data;
}

