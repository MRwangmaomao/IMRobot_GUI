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
#include <riki_msgs/grasptask.h>

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
    void pub_grasp_task();
Q_SIGNALS:
    void loggingUpdated();
    void loggingListen();
    void rosShutdown();
    void batteryUpdated(float);

private:
    int init_argc;
    char** init_argv;


    // cmd vel
    ros::Publisher velcmd_publisher;

    // grasp_start
    ros::Publisher grasp_start_publisher;

    // grasp_ok
    ros::Subscriber grasp_stop_subscriber;

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
