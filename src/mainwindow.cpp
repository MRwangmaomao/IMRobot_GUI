#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSettings>
#include <QMessageBox>
#include <QAction>
#include <QIcon>
MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    qnode(argc,argv)
{
    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(2);


//    QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
//    ui->tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ReadSettings();
    /*********************
    ** Logging
    **********************/
    ui->listViewLog->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    ui->listViewListen->setModel(qnode.loggingModelLis());
    QObject::connect(&qnode, SIGNAL(loggingListen()), this, SLOT(updateLogListen()));

    connect(ui->actionimu, &QAction::triggered,this,[=](){
        ui->stackedWidget->setCurrentIndex(1);
    });

    connect(ui->actionpid_set, &QAction::triggered, this, [=](){
        ui->stackedWidget->setCurrentIndex(0);
    });

    connect(ui->actionwelcome,&QAction::triggered,this,[=](){
        ui->stackedWidget->setCurrentIndex(2);
    });


    /*********************
    ** Auto Start
    **********************/
    if(ui->checkbox_remember_settings->isChecked())
    {
        on_checkbox_use_environment_stateChanged(true);
    }

    /********************
    ** Update
    *********************/
    QObject::connect(&qnode, &QNode::batteryUpdated, this, &MainWindow::updatabatterydata);
}

MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
    ui->listViewLog->scrollToBottom();
}

void MainWindow::updateLogListen()
{
    ui->listViewListen->scrollToBottom();
}


void MainWindow::on_pushButtonConnect_clicked()
{
    qnode.init();
}

void MainWindow::on_pushButtonStop_clicked()
{
    qnode.stop_thubot();
}

void MainWindow::on_pushButtonQuit_clicked()
{
    this->close();
}


void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "test_gui");
    settings.setValue("master_url",ui->line_edit_master->text());
    settings.setValue("host_url",ui->line_edit_host->text());
    //settings.setValue("topic_name",ui->line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui->checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui->checkbox_remember_settings->isChecked()));
}


void MainWindow::on_checkbox_use_environment_stateChanged(int arg1)
{
    if(ui->checkbox_use_environment->isChecked())
    {
        if(!qnode.init())
            showNoMasterMessage();
        else
            ui->pushButtonConnect->setEnabled(false);
    }
    else
    {
        if(!qnode.init(ui->line_edit_master->text().toStdString(),
                          ui->line_edit_host->text().toStdString()))
        {
            showNoMasterMessage();
        }
        else
        {
            ui->pushButtonConnect->setEnabled(false);
            ui->line_edit_master->setReadOnly(true);
            ui->line_edit_host->setReadOnly(true);
            //ui->line_edit_topic->setReadOnly(true);
        }
    }
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/
void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "test_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui->line_edit_master ->setText(master_url);
    ui->line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui->checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui->checkbox_use_environment->setChecked(checked);
    if(checked)
    {
        ui->line_edit_master->setEnabled(false);
        ui->line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::showNoMasterMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}


void MainWindow::on_pushButtonUp_clicked()
{
    qnode.up();
}

void MainWindow::on_pushButtonLeft_clicked()
{
    qnode.left();
}

void MainWindow::on_pushButtonRight_clicked()
{
    qnode.right();
}

void MainWindow::on_pushButtonDown_clicked()
{
    qnode.down();
}

void MainWindow::on_pushButtonThubot_clicked()
{
    system("gnome-terminal -x zsh -c 'source ~/IM-robot/devel/setup.zsh;roslaunch thurobot bringup.launch  limited:=true'&");
}

void MainWindow::on_pushButtonIMUdatasave_clicked()
{
    current_date_time =QDateTime::currentDateTime();
    QString current_date =current_date_time.toString("yyyy.MM.dd hh:mm:ss.zzz ddd");
    qDebug() << current_date;
}


void MainWindow::updatabatterydata(float value)
{
    ui->label_battary_arm->setText(QString("%3V").arg(value));
}

void MainWindow::on_pushButtonSavMap_clicked()
{

}

void MainWindow::on_pushButtonLoadMap_clicked()
{

}

void MainWindow::on_pushButtonGrasp_clicked()
{
    qnode.pub_grasp_task();
}
