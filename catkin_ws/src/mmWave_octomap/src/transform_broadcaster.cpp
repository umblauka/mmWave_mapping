#include "serial/serial.h"
#include <mmWave_octomap/transform_broadcaster.hpp>

#include <ros/console.h>

#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <ros/assert.h>
#include <signal.h>

int dataBaudRate = 115200;
serial::Serial mySerialObject("/dev/ttyTHS0", dataBaudRate, serial::Timeout::simpleTimeout(100));

std::vector<std::string> split(std::string str, char sep = ' ')
{
    std::vector<std::string> ret;

    std::istringstream stm(str);
    std::string token;
    while (std::getline(stm, token, sep))
        ret.push_back(token);

    return ret;
}
void my_handler(int s)
{
    if (mySerialObject.isOpen())
    {
        mySerialObject.close();
        exit(1);
    }
}
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        return -1;
    }
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    ROSCONSOLE_AUTOINIT;
    ros::init(argc, argv, "tf2_broadcaster");
    //set ros console to output to terminal
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    { // Change the level to fit your needs
        ros::console::notifyLoggerLevelsChanged();
    }
    if (mySerialObject.isOpen())
        ROS_DEBUG("Port opened");
    else
        ROS_ERROR("Port could not be opened. Port is \"%s\" and baud rate is %d", "/dev/ttyACM0", dataBaudRate);
    std::size_t maxSizeOfLine = 256;

    while (1)
    {
        sigaction(SIGINT, &sigIntHandler, NULL);
        std::string result = mySerialObject.readline(maxSizeOfLine, "\n");
        std::string sizeOfRecievedLine = std::to_string(result.size());
        //ROS_DEBUG("%s %s", result.c_str(), result.empty() ? "empty" : sizeOfRecievedLine.c_str());

        if (!result.empty() && result.size() > 10)
        {
            std::vector<std::string> tokens = split(result, ' ');
            //ROS_DEBUG("lX: %s, lY: %s, lZ: %s, W: %s, X: %s, Y: %s, Z: %s, ID: %s", tokens[0].c_str(), tokens[1].c_str(), tokens[2].c_str(), tokens[3].c_str(), tokens[4].c_str(), tokens[5].c_str(), tokens[6].c_str(), tokens[7].c_str());
            geometry_msgs::Quaternion quat;
            quat.w = std::stof(tokens[3]);
            quat.x = std::stof(tokens[4]);
            quat.y = std::stof(tokens[5]);
            quat.z = std::stof(tokens[6]);
            if (!std::isnan(quat.w))
            {

                tf2::Quaternion myQuaternion(stof(tokens[4]), stof(tokens[5]), stof(tokens[6]), stof(tokens[3]));
                myQuaternion.normalize();

                static tf2_ros::TransformBroadcaster br;
                geometry_msgs::TransformStamped transformStamped;

                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "map";
                transformStamped.child_frame_id = "base_link";
                transformStamped.transform.translation.x = 2.0;
                transformStamped.transform.translation.y = 0.0;
                transformStamped.transform.translation.z = 0.0;
                transformStamped.transform.rotation.x = myQuaternion.x();
                transformStamped.transform.rotation.y = myQuaternion.y();
                transformStamped.transform.rotation.z = myQuaternion.z();
                transformStamped.transform.rotation.w = myQuaternion.w();

                br.sendTransform(transformStamped);

                geometry_msgs::TransformStamped transformStamped1;
                tf2::Quaternion radar1Quat(0, 0, 0.7071, 0.7071);
                radar1Quat.normalize();

                transformStamped1.header.stamp = ros::Time::now();
                transformStamped1.header.frame_id = "base_link";
                transformStamped1.child_frame_id = "base_radar_link_1";
                transformStamped1.transform.translation.x = 0.25;
                transformStamped1.transform.translation.y = 0.25;
                transformStamped1.transform.translation.z = 0.0;
                transformStamped1.transform.rotation.x = 0;
                transformStamped1.transform.rotation.y = 0;
                transformStamped1.transform.rotation.z = radar1Quat.z();
                transformStamped1.transform.rotation.w = radar1Quat.w();

                br.sendTransform(transformStamped1);

                geometry_msgs::TransformStamped transformStamped2;
                tf2::Quaternion radar2Quat(0, 0, 0.7071, 0.7071);
                radar2Quat.normalize();

                transformStamped2.header.stamp = ros::Time::now();
                transformStamped2.header.frame_id = "base_link";
                transformStamped2.child_frame_id = "base_radar_link_2";
                transformStamped2.transform.translation.x = 0.25;
                transformStamped2.transform.translation.y = 0.0;
                transformStamped2.transform.translation.z = 0.0;
                transformStamped2.transform.rotation.x = 0;
                transformStamped2.transform.rotation.y = 0;
                transformStamped2.transform.rotation.z = radar2Quat.z();
                transformStamped2.transform.rotation.w = radar2Quat.w();

                br.sendTransform(transformStamped2);

                //ROS_DEBUG("W:%f X:%f Y:%f Z:%f", quat.w, quat.x, quat.y, quat.z);
            }
        }
        sigaction(SIGINT, &sigIntHandler, NULL);
    }
    return 0;
}
