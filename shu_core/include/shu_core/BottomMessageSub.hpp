#ifndef BOTTOMMESSAGESUB_HPP
#define BOTTOMMESSAGESUB_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

class BottomMessageSub{
 public:
    BottomMessageSub();
    ~BottomMessageSub() = default;
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 private:
    ros::NodeHandle nh_;

    ros::Subscriber pos_sub;
    ros::Subscriber imu_sub;

    sensor_msgs::Imu imu_msg;

    double position[3] = {0,0,0};

};

#endif