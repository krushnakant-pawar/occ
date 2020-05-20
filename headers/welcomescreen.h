#ifndef WELCOMESCREEN_H
#define WELCOMESCREEN_H

#include <QDialog>
#include "mainwindow.h"
namespace Ui {
class welcomeScreen;
}

class welcomeScreen : public QDialog
{
    Q_OBJECT

public:
    explicit welcomeScreen(QWidget *parent = nullptr);
    ~welcomeScreen();

private slots:
    void on_pushButton_continue_clicked();

private:
    Ui::welcomeScreen *ui;
    MainWindow *mainWindow;
};

#endif // WELCOMESCREEN_H
