/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <qnode.h>
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace test_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv )
    : init_argc(argc), init_argv(argv)
{}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"gui");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

    velcmd_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    battery_subscriber = n.subscribe("/battery",100,&QNode::RecvBatteryTopicCallback, this);
    grasp_start_publisher = n.advertise<riki_msgs::grasptask>("/grasp_start",10);

    //ros::spin();
    qDebug() << "I start";
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"test_gui");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    //chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    //chatter_subscriber = n.subscribe("testgui_chat", 100, &QNode::RecvTopicCallback, this);
    //ros::spin();
    start();
    return true;
}

void QNode::RecvBatteryTopicCallback(const riki_msgs::BatteryConstPtr &msg)
{
    float value = msg->battery;
    Q_EMIT batteryUpdated(value);
}

void QNode::run()
{
    ros::Rate loop_rate(1);
    ros::NodeHandle n;
    velcmd_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    grasp_start_publisher = n.advertise<riki_msgs::grasptask>("/grasp_start",10);
    battery_subscriber = n.subscribe("/battery",100,&QNode::RecvBatteryTopicCallback, this);
    ros::spin();
    loop_rate.sleep();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::ros_test(const std::string s)
{
    ROS_INFO_STREAM(s);
}

void QNode::log(const LogLevel &level, const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch (level)
    {
        case(Debug):
        {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Info):
        {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Warn):
        {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Error):
        {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Fatal):
        {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_listen(const LogLevel &level, const std::string &msg)
{
    logging_listen.insertRows(logging_listen.rowCount(),1);
    std::stringstream logging_model_msg;
    switch (level)
    {
        case(Debug):
        {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Info):
        {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Warn):
        {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Error):
        {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
        }
        case(Fatal):
        {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_listen.setData(logging_listen.index(logging_listen.rowCount()-1),new_row);
    Q_EMIT loggingListen(); // used to readjust the scrollbar
}

void QNode::up()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 2.0;
    msg.angular.z = 0.0;
    velcmd_publisher.publish(msg);

    std::stringstream ss;
    std_msgs::String logmsg;
    ss << "up x= 2.0, z = 0.0";
    logmsg.data = ss.str();
    log(Info,std::string("I sent: ")+logmsg.data);

}

void QNode::down()
{
    geometry_msgs::Twist msg;
    msg.linear.x = -2.0;
    msg.angular.z = 0.0;
    velcmd_publisher.publish(msg);

    std::stringstream ss;
    std_msgs::String logmsg;
    ss << "Down x= -2.0, z = 0.0";
    logmsg.data = ss.str();
    log(Info,std::string("I sent: ")+logmsg.data);
}

void QNode::left()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 2.0;
    msg.angular.z = 1.57;
    velcmd_publisher.publish(msg);

    std::stringstream ss;
    std_msgs::String logmsg;
    ss << "Down x= 2.0, z = 1.57";
    logmsg.data = ss.str();
    log(Info,std::string("I sent: ")+logmsg.data);
}

void QNode::right()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 2.0;
    msg.angular.z = -1.57;
    velcmd_publisher.publish(msg);

    std::stringstream ss;
    std_msgs::String logmsg;
    ss << "up x= 2.0, z = -1.57";
    logmsg.data = ss.str();
    log(Info,std::string("I sent: ")+logmsg.data);
}

void QNode::stop_thubot()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velcmd_publisher.publish(msg);
}

void QNode::pub_grasp_task()
{
    riki_msgs::grasptask msg;
    msg.isstart = true;
    grasp_start_publisher.publish(msg);
    qDebug() << "start to grasp";
}

}  // namespace test_gui
