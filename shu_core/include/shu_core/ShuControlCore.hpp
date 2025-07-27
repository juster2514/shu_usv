#ifndef SHUCONTROLCORE_HPP
#define SHUCONTROLCORE_HPP

#include "shu_core/MixedControl.hpp"
#include "shu_core/UsvPid.hpp"
#include "shu_core/TraditionalLos.hpp"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class ShuControlCore{
 public:

    ShuControlCore();
    ~ShuControlCore() = default;

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void mavros_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void tUpdate();

    geometry_msgs::PoseStamped getPosition() const { return position; };//外部获取位姿的函数


 private:

    double roll_current, pitch_current, yaw_current;

    ros::NodeHandle nh_;

    std::shared_ptr<UsvPid> usv_pid_ptr_;
    std::shared_ptr<MixedControl> mixed_control_ptr_;
    std::shared_ptr<TraditionalLos> usv_los_ptr_;

    sensor_msgs::Imu imu_msg;
    geometry_msgs::PoseStamped position;
    geometry_msgs::TwistStamped velocity;

    Eigen::Vector2d segStart;
	 Eigen::Vector2d segEnd;
	 Eigen::Vector2d position_2D;

    ros::Publisher actuator_control_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber imu_sub;

};

#endif