#ifndef SHUCONTROLCORE_HPP
#define SHUCONTROLCORE_HPP

#include "shu_core/MixedControl.hpp"
#include "shu_core/UsvPid.hpp"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class ShuControlCore{
 public:

    ShuControlCore();
    ~ShuControlCore() = default;

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped getPosition() const { return position; };

    void tUpdate();


 private:

    double roll, pitch, yaw;

    ros::NodeHandle nh_;

    std::shared_ptr<UsvPid> usv_pid_ptr_;
    std::shared_ptr<MixedControl> mixed_control_ptr_;

    sensor_msgs::Imu imu_msg;
    geometry_msgs::PoseStamped position;
    geometry_msgs::TwistStamped velocity;

    ros::Publisher actuator_control_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber imu_sub;

};

#endif