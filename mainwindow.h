#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QMovie>
#include <worker.h>
#include <QMap>
#include <messages.h>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    QMap<uint8_t, MsgFormat> msgDir;
    void initMsgDir(){
        msgDir[0b11101000] = {1, "ACCIDENT AHEAD!", ":/img/accident-anim.gif"};
        msgDir[0b00011100] = {2, "SLOWING DOWN!", ":/img/white.png"};
        msgDir[0b01100100] = {3, "CONSTRUCTION AHEAD!", ":/img/white.png"};
        msgDir[0b11011011] = {4, "Overtaking allowed", ":/img/white.png"};
        msgDir[0b10000000] = {5, "MSG 5", ":/img/white.png"};
        msgDir[0b00010111] = {6, "MSG 6", ":/img/white.png"};
        msgDir[0b10110110] = {7, "MSG 7", ":/img/white.png"};
        msgDir[0b00001011] = {8, "MSG 8", ":/img/white.png"};
        msgDir[0b00000000] = {255, "NO Message", ":/img/white.png"};
    }
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();    
private slots:
    void updateWorkerUI(QImage img, QVector<uint8_t> msg,int frameDelay, int processingDelay);

    void on_pushButton_play_clicked();
private:
    Ui::MainWindow *ui;
    Worker *worker;
    QMovie *movie;
};
#endif // MAINWINDOW_H
