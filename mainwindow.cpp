#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTextStream>
#include <QTimer>
#include <QDebug>
#include <bitset>
#define MAX_MSGS 5
extern bool detect;
extern QMutex detectMutex;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    worker = new Worker();
    initMsgDir();
    connect(worker, SIGNAL(processedImage(QImage, QVector<uint8_t>, int, int)), this, SLOT(updateWorkerUI(QImage, QVector<uint8_t>, int, int)));

    QFile::copy(":/files/car-tail-lamp-cascade-20stages.xml", "/tmp/car-tail-lamp-cascade-20stages.xml");

    ui->setupUi(this);
    QTimer::singleShot(0, this, SLOT(showFullScreen()));
    ui->label_msg->setText("");
    ui->label_msg_img->setPixmap(QPixmap(":/img/white.png").scaled(ui->label_msg_img->size()));
    ui->label_lamp->setPixmap(QPixmap(":/img/black.jpg").scaled(ui->label_lamp->size()));
    ui->label_dese->setPixmap(QPixmap(":/img/dese.png").scaled(ui->label_dese->size()));
    ui->label_iisc->setPixmap(QPixmap(":/img/iisc.gif").scaled(ui->label_iisc->size()));
}

MainWindow::~MainWindow()
{
    qDebug() << "Main window destructor called" << endl;
    delete worker;
    delete movie;
    delete ui;
}

bool compPriorities(MsgFormat a, MsgFormat b){
    return a.prio < b.prio;
}

void MainWindow::updateWorkerUI(QImage img, QVector<uint8_t> msg, int frameDelay, int processingDelay)
{

    if(!img.isNull())
    {
        ui->label_lamp->setAlignment(Qt::AlignHCenter);
        ui->label_lamp->setPixmap(QPixmap::fromImage(img).scaled(ui->label_lamp->size(),Qt::KeepAspectRatio, Qt::FastTransformation));
        ui->label_msg->setAlignment(Qt::AlignHCenter);
        float fps = 1000.0/frameDelay;
        QString fps_string;
        QTextStream in(&fps_string);
        in << "Frame Rate(FPS): " << std::trunc(fps) << "\tDelay(ms): " << processingDelay;
        ui->label_fps->setText(fps_string);
        int sum = std::accumulate(msg.begin(), msg.end(), 0);
        if(sum != 0)
        {
            //cout << std::bitset<8>(msg.toStdVector()[0]) << "\t" << std::bitset<8>(msg.toStdVector()[1]) << endl;
            QString textMsg = "";
            MsgFormat allMsgs[msg.size()];

            for(int i = 0; i < msg.size(); i++){
                allMsgs[i] = msgDir[msg[i]];
            }

            std::sort(allMsgs, allMsgs + msg.size(), compPriorities);

            int criticalPriority = allMsgs[0].prio;
            QString imgPath = allMsgs[0].imgPath;
            textMsg = allMsgs[0].msg;

            if(criticalPriority == 1)
            {

                ui->label_msg->setText(textMsg);
                ui->label_msg_img->setPixmap(QPixmap(imgPath).scaled(ui->label_msg_img->size()));
                movie = new QMovie(imgPath);
                QSize size1 = ui->label_msg_img->size();
                movie->setScaledSize(size1);

                ui->label_msg_img->setAlignment(Qt::Alignment(Qt::AlignCenter));
                ui->label_msg_img->setMovie(movie);
                movie->start();

            }
            else
            {
                ui->label_msg->setText("");
                ui->label_msg_img->setPixmap(QPixmap(":/img/white.png").scaled(ui->label_msg_img->size()));
            }

        }
    }
}


void MainWindow::on_pushButton_play_clicked()
{
    if(worker->isStopped())
    {
        worker->initStream();
        worker->startWorker();
        ui->pushButton_play->setText("Quit");
        ui->pushButton_play->setStyleSheet("background-color:rgb(255, 0, 0)");
    }else
    {
        worker->stopWorker();
        ui->pushButton_play->setText("Start");
        ui->pushButton_play->setStyleSheet("background-color:rgb(90, 250, 28)");
        ui->label_msg->setText("");
        ui->label_msg_img->setPixmap(QPixmap(":/img/white.png").scaled(ui->label_msg_img->size()));
        ui->label_lamp->setPixmap(QPixmap(":/img/black.jpg").scaled(ui->label_lamp->size()));
        detectMutex.lock();
        detect = true;
        detectMutex.unlock();
        this->close();        
        QWidget::parentWidget()->show();
    }
}

