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

struct PointPath {
    PointPath(const std::string& point_parmas_file) {
        cv::FileStorage file(point_parmas_file, cv::FileStorage::READ);

        std::vector<double> temp_x, temp_y;
        file["point_x"] >> temp_x;
        file["point_y"] >> temp_y;
        
        points.reserve(temp_x.size());
        for (size_t i = 0; i < temp_x.size(); ++i) {
            points.emplace_back(temp_x[i], temp_y[i]);
        }
        points_size = static_cast<int>(points.size());
    }

    // 访问所有点的引用
    const std::vector<Eigen::Vector2d>& getPoints() const { return points; }

    // 访问单个点
    Eigen::Vector2d getPoint(size_t index) const { return points[index]; }

    int getPointSize(){ return points_size;}

private:
    std::vector<Eigen::Vector2d> points;  // 存储所有二维点

    int points_size;
};


class ShuSimCore{
 public:

    ShuSimCore();
    ~ShuSimCore() = default;

    void shuSimVisualizationInit();

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void modelLeftVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void modelRightVelCallback(const std_msgs::Float32::ConstPtr& msg);

    void velPublishSet(std_msgs::Float32 vel_set);
    void anglePublishSet(std_msgs::Float32 angle_set);
    float calculateAngleError(float current, float target);

    void tUpdate();

 private:
 
    double roll_current, pitch_current, yaw_current;
    double left_speed_current, right_speed_current;

    ros::NodeHandle nh_;

    std::shared_ptr<TraditionalLosSim> usv_los_sim_ptr_;
    std::shared_ptr<UsvSimPid> usv_sim_pid_ptr_;
    std::shared_ptr<PointPath> usv_sim_path_ptr_;

    LosSimOut los_result;

    geometry_msgs::Pose wamv_pose;
    geometry_msgs::Twist wamv_twist;
    geometry_msgs::PoseStamped pose_stamped;

    nav_msgs::Path path_msg;

    visualization_msgs::Marker line_marker;
 
    std_msgs::Float32 vel_value;
    std_msgs::Float32 angle_value;

    std::vector<Eigen::Vector2d> full_path_points;

    Eigen::Vector2d segStart;
	Eigen::Vector2d segEnd;
    Eigen::Vector2d position_2D;

    ros::Publisher left_vel_control;
    ros::Publisher right_vel_control;

    ros::Publisher left_angle_control;
    ros::Publisher right_angle_control;

    ros::Publisher model_pose;
    ros::Publisher reference_line;

    ros::Subscriber usv_pose;  
    ros::Subscriber usv_left_vel;  
    ros::Subscriber usv_right_vel;  

};

#endif