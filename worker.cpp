#include "worker.h"
#include <QTime>
#include <QDebug>
#include <bitset>
extern QMutex processMutex;
Worker::Worker(QObject *parent) : QThread(parent)
{
    stop = true;
}

bool Worker::initStream()
{

    //capture.open(0);
    //capture.open("/home/krushnakant/QT_creator_workspace/sunlight_fsk_50k_hf_50_45hz_data_11101000.avi");
    //capture.open("/home/krushnakant/QT_creator_workspace/movement_fsk_50k_hf_50_45hz_data_11101000.avi");
    capture.open("/home/krushnakant/QT_creator_workspace/multi_lamp_different_zoom.webm");
    if(capture.isOpened())
    {
        frameRate = capture.get(CV_CAP_PROP_FPS);
        qDebug() << "Frame rate: " << frameRate << endl;
        return true;
    }
    else
        return false;
}

void Worker::startWorker()
{
    if(!isRunning()){
        if(isStopped()){
            mutex.lock();
            stop = false;
            mutex.unlock();
        }
    start();
    }
}

void Worker::run()
{
    int framePeriod = (1000/frameRate);
    static int timePerFrame = 0; // in ms
    while(!stop)
    {
        if(!capture.read(frame))
        {
            mutex.lock();
            stop = true;
            mutex.unlock();
        }
        if(!frame.empty())
        {
            QTime t1;
            t1.restart();
            QTime t;
            t.restart();
            DECODED_DATA decode_data = processFrame(frame);
            processMutex.lock();

            Mat lamp = decode_data.img;
            cv::cvtColor(lamp, RGBframe, CV_BGR2RGB);

            img = QImage((const unsigned char*)(RGBframe.data)
                         , lamp.cols, lamp.rows, QImage::Format_RGB888);
            int lapsed = t.elapsed();
            QVector<uint8_t> msg;
            msg.clear();
            for(int i = 0; i< decode_data.msg.size(); i++){
                msg.push_back(decode_data.msg[i]);
            }

            // register to emit the QVector type over queued connections
            qRegisterMetaType<QVector<uint8_t> >("QVector<uint8_t>");
            emit processedImage(img, msg, timePerFrame, lapsed);
            processMutex.unlock();
            //std::cout << "time elpased: " << lapsed;
            int delay = framePeriod - lapsed;
            //std::cout << "\tdelay: " << delay;
            if(delay > 0)
                this->msleep(delay);
            timePerFrame = t1.elapsed();
            //std::cout << "\ttotal time: " << timePerFrame << std::endl;
        }
    }
}

Worker::~Worker()
{
    qDebug() << "worker destructor called" << endl;
    exit(0);
}

void Worker::stopWorker()
{
    mutex.lock();
    stop = true;
    mutex.unlock();
    capture.release();
    wait();
    quit();
}

void Worker::msleep(int ms)
{
    struct timespec ts = {ms /1000, (ms %1000) * 1000*1000};
    nanosleep(&ts, NULL);
}

bool Worker::isStopped() const{
    return this->stop;
}
