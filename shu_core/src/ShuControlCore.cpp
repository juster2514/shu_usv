#include "shu_core/ShuControlCore.hpp"

extern bool clean_integral;

/*
*描述：控制核心构造函数
*作用：初始化系统
*参数：无
*输出：无
*/
ShuControlCore::ShuControlCore():nh_("~"){

//测试时使用，正式实验时需换成母船拟合的直线
	segStart[0] = 10.0; 
	segStart[1] = 20.0; 
	segEnd[0] = 40.0; 
	segEnd[1] = 60.0; 

//初始化对象
    usv_pid_ptr_ = std::make_shared<UsvPid>();
    mixed_control_ptr_ = std::make_shared<MixedControl>();
	usv_los_ptr_ = std::make_shared<TraditionalLOS>();

//初始化发布话题
	ros::Publisher actuator_control_pub = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",10);

//初始化订阅话题
	ros::Subscriber pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&ShuControlCore::mavros_pos_callback,this);
	ros::Subscriber vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,&ShuControlCore::vel_callback,this);
    ros::Subscriber imu_sub = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,&ShuControlCore::imu_callback,this);

//初始化线程
	std::thread update = std::thread(&ShuControlCore::tUpdate, this);
	update.detach();

}

/*
*描述：回收过程第一阶段的位姿回调函数
*作用：对回收第一阶段USV位姿的获取和更新
*参数：无
*输出：无
*/
void ShuControlCore::mavros_pos_callback (const geometry_msgs::PoseStamped::ConstPtr& msg) { 

	position = *msg; 
	position_2D[0] = position.pose.position.x;
	position_2D[1] = position.pose.position.y;

	geometry_msgs::Quaternion qtn_ = msg->pose.orientation;
	tf2::Matrix3x3 m(tf2::Quaternion(qtn_.x, qtn_.y, qtn_.z, qtn_.w));
    m.getRPY(roll_current,pitch_current,yaw_current);

}

/*
*描述：回收过程第一阶段的速度回调函数
*作用：对回收第一阶段USV速度的获取和更新
*参数：无
*输出：无
*/
void ShuControlCore::vel_callback (const geometry_msgs::TwistStamped::ConstPtr& msg) { velocity = *msg; }

/*
*描述：回收过程的IMU回调函数
*作用：对回收过程IMU数据的获取
*参数：无
*输出：无
*/
void ShuControlCore::imu_callback (const sensor_msgs::Imu::ConstPtr& msg) { imu_msg = *msg; }

/*
*描述：控制核心的更新线程函数
*作用：作为控制核心的主要决策函数，以及实时的任务需求更新和发布
*参数：无
*输出：无
*/
void ShuControlCore::tUpdate(){

	while (ros::ok()) {

		// float target_yaw = (*usv_los_ptr_)(segStart,segEnd,position_2D,usv_los_ptr_->getLos());
		// float pid_angle_out = (*usv_pid_ptr_)(target_yaw,yaw_current,usv_pid_ptr_->getAnglePid(),clean_integral);

		// mixed_control_ptr_->actuator_set(pid_angle_out,0.1);
		// actuator_control_pub.publish(mixed_control_ptr_->getActuator());
	}

}