/**
 * @file /include/test_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2018
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef test_gui_QNODE_HPP_
#define test_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <QThread>
#include <QStringListModel>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <riki_msgs/Velocities.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace test_gui
{

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    /*********************
    ** Logging
    **********************/
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel* loggingModel()
        {return &logging_model;}
    void log( const LogLevel &level, const std::string &msg);

    //void RecvTopicCallback(const std_msgs::StringConstPtr &msg);

    void RecvBatteryTopicCallback(const riki_msgs::BatteryConstPtr &msg);


    QStringListModel* loggingModelLis()
        {return &logging_listen;}
    void log_listen(const LogLevel &level, const std::string &msg);

    void ros_test(const std::string s);

    void up();
    void down();
    void left();
    void right();
    void stop_thubot();
Q_SIGNALS:
    void loggingUpdated();
    void loggingListen();
    void rosShutdown();
    void batteryUpdated(float);

private:
    int init_argc;
    char** init_argv;
    ros::Publisher velcmd_publisher;

    // cmd vel
    ros::Publisher cmd_vel_subscriber;

    // IMU
    ros::Subscriber IMU_mag_subscriber;
    ros::Subscriber IMU_data_subscriber;
    ros::Subscriber IMU_filter_madgwick_param_subscriber;

    // odom
    ros::Subscriber odom_subscriber;

    // battery
    ros::Subscriber battery_subscriber;

    // pid
    ros::Subscriber pid_subscriber;

    // tf
    ros::Subscriber tf_subscriber;

    QStringListModel logging_model;
    QStringListModel logging_listen;
};

}  // namespace test_gui

#endif /* test_gui_QNODE_HPP_ */
