#include "shu_core/BottomMessageSub.hpp"

BottomMessageSub::BottomMessageSub():nh_("~"){
    
    ros::Subscriber pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&BottomMessageSub::pos_callback,this);
    ros::Subscriber imu_sub = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,&BottomMessageSub::imu_callback,this);
	
}

void BottomMessageSub::pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	position[0] = msg->pose.position.x;
	position[1] = msg->pose.position.y;
	position[2] = msg->pose.position.z;
}

void BottomMessageSub::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
	imu_msg = *msg;
}
