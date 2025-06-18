#include "shu_core/ShuControlCore.hpp"

ShuControlCore::ShuControlCore():nh_("~"){

    usv_pid_ptr_ = std::make_shared<UsvPid>();
    mixed_control_ptr_ = std::make_shared<MixedControl>();

	ros::Publisher actuator_control_pub = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",10);

	ros::Subscriber pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&ShuControlCore::pos_callback,this);
	ros::Subscriber vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,&ShuControlCore::vel_callback,this);
    ros::Subscriber imu_sub = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,&ShuControlCore::imu_callback,this);

	std::thread update = std::thread(&ShuControlCore::tUpdate, this);
	update.detach();

}

void ShuControlCore::pos_callback (const geometry_msgs::PoseStamped::ConstPtr& msg) { 
	position = *msg; 
	geometry_msgs::Quaternion qtn_ = msg->pose.orientation;
	tf2::Matrix3x3 m(tf2::Quaternion(qtn_.x, qtn_.y, qtn_.z, qtn_.w));
    m.getRPY(roll,pitch,yaw);
}

void ShuControlCore::vel_callback (const geometry_msgs::TwistStamped::ConstPtr& msg) { velocity = *msg; }

void ShuControlCore::imu_callback (const sensor_msgs::Imu::ConstPtr& msg) { imu_msg = *msg; }

void ShuControlCore::tUpdate(){
	while (ros::ok()) {

	}
}