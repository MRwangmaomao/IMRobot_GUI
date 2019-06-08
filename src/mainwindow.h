#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qnode.h>
#include <QDateTime>
#include <QDebug>

using namespace test_gui;

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

private slots:
    void on_pushButtonConnect_clicked();

    void on_pushButtonStop_clicked();

    void on_pushButtonQuit_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateLogListen();
    void updatabatterydata(float);
//    void updateIMUDataView();


    void on_checkbox_use_environment_stateChanged(int arg1);

    void on_pushButtonUp_clicked();

    void on_pushButtonLeft_clicked();

    void on_pushButtonRight_clicked();

    void on_pushButtonDown_clicked();

    void on_pushButtonThubot_clicked();

    void on_pushButtonIMUdatasave_clicked();

    void on_pushButtonSavMap_clicked();

    void on_pushButtonLoadMap_clicked();

    void on_pushButtonGrasp_clicked();

private:
    Ui::MainWindow *ui;
    QNode qnode;
    QDateTime current_date_time;
};

#endif // MAINWINDOW_H
