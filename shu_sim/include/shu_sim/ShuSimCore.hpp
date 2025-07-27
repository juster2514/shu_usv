#ifndef SHUSIMCORE_HPP
#define SHUSIMCORE_HPP

#include <memory>
#include <thread>

#include "shu_sim/TraditionalLosSim.hpp"
#include "shu_sim/UsvSimPid.hpp"
#include <gazebo_msgs/ModelStates.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class ShuSimCore{
 public:

    ShuSimCore();
    ~ShuSimCore() = default;

    void shuSimCoreInit();

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    void velPublishSet(std_msgs::Float32 vel_set);
    void anglePublishSet(std_msgs::Float32 angle_set);

    void tUpdate();

 private:
 
    double roll_current, pitch_current, yaw_current;

    ros::NodeHandle nh_;

    std::shared_ptr<TraditionalLosSim> usv_los_sim_ptr_;
    std::shared_ptr<UsvSimPid> usv_sim_pid_ptr_;

    geometry_msgs::Pose wamv_pose;
    geometry_msgs::Twist wamv_twist;

    nav_msgs::Path path_msg;

    visualization_msgs::Marker line_marker;
 
    std_msgs::Float32 vel_value;
    std_msgs::Float32 angle_value;

    Eigen::Vector2d segStart;
	 Eigen::Vector2d segEnd;
    Eigen::Vector2d position_2D;

    ros::Publisher left_vel_control;
    ros::Publisher right_vel_control;

    ros::Publisher left_angle_control;
    ros::Publisher right_angle_control;

    ros::Publisher model_pose;
    ros::Subscriber usv_pose;  

    ros::Publisher reference_line;

};

#endif