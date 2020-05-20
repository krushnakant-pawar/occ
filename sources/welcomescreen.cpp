#include "welcomescreen.h"
#include "ui_welcomescreen.h"
#include <QMovie>
welcomeScreen::welcomeScreen(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::welcomeScreen)
{
    ui->setupUi(this);
    ui->label_dese->setPixmap(QPixmap(":/img/dese.png").scaled(ui->label_dese->size()));
    ui->label_iisc->setPixmap(QPixmap(":/img/iisc.gif").scaled(ui->label_iisc->size()));
    QMovie *welcome = new QMovie(":/img/title.gif");
    welcome->setScaledSize(ui->label_welcome->size());
    ui->label_welcome->setMovie(welcome);
    welcome->start();
}

welcomeScreen::~welcomeScreen()
{
    delete ui;
    delete mainWindow;
}

void welcomeScreen::on_pushButton_continue_clicked()
{
    if(mainWindow == nullptr)
        mainWindow = new MainWindow(this);
    mainWindow->setWindowState(mainWindow->windowState()|Qt::WindowFullScreen);
    hide();
    mainWindow->show();
}
