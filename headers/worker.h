#ifndef WORKER_H
#define WORKER_H
#include <QMutex>
#include <QThread>
#include <QImage>
#include <QWaitCondition>
#include <occ.h>


class Worker : public QThread
{
    Q_OBJECT
private:
    bool stop;
    QMutex mutex;
    Mat frame;
    int frameRate;
    VideoCapture capture;
    Mat RGBframe;
    QImage img;

signals:
    void processedImage(const QImage &image, const QVector<uint8_t> msg, const int timePerFrame, const int processingDelay);

protected:
    void run();
    void msleep(int ms);
public:
    Worker(QObject *parent = 0);
    ~Worker();
    bool initStream();
    void startWorker();
    void stopWorker();
    bool isStopped() const;
};
#endif // WORKER_H
