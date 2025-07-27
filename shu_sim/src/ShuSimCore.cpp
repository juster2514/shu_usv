#include "shu_sim/ShuSimCore.hpp"
/*
*描述：ShuSimCore类的构造函数
*作用：初始化仿真系统
*参数：无
*输出：无
*/
ShuSimCore::ShuSimCore():nh_("~"){

    segStart[0] = -40.0; 
	segStart[1] = 0.0; 
	segEnd[0] = 160.0; 
	segEnd[1] = 80.0; 

//初始化对象
	usv_los_sim_ptr_ = std::make_shared<TraditionalLosSim>();
    usv_sim_pid_ptr_ = std::make_shared<UsvSimPid>();

//初始化发布话题
    left_vel_control = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd",10);
    right_vel_control = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd",10);

    left_angle_control = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle",10);
    right_angle_control = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle",10);

    model_pose = nh_.advertise<nav_msgs::Path>("/usv_sim_pose",10);
    reference_line = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

//初始化订阅话题
    usv_pose = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,&ShuSimCore::modelStatesCallback,this);

//初始化可视化参数
    shuSimCoreInit();

//仿真更新循环
    tUpdate();
}

/*
*描述：可视化参数初始化函数
*作用：设置一些可视化参数
*参数：无
*输出：无
*/
void ShuSimCore::shuSimCoreInit(){

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    line_marker.pose.orientation.w = 1.0;

    line_marker.scale.x = 0.3; // 线宽

    line_marker.color.r = 1.0;
    line_marker.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    p1.x = segStart[0]; p1.y = segStart[1]; p1.z = 0.0;
    p2.x = segEnd[0]; p2.y = segEnd[1]; p2.z = 0.0;

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
}

/*
*描述：仿真模型的位姿信息回调函数
*作用：更新模型实时位姿
*参数：无
*输出：无
*/
void ShuSimCore::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // 遍历所有模型名称
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "wamv") {  // 检查是否为wamv模型
            // 提取位置和姿态
            wamv_pose = msg->pose[i];

            wamv_twist = msg->twist[i];

	        position_2D[0] = wamv_pose.position.x;
	        position_2D[1] = wamv_pose.position.y;

            geometry_msgs::Quaternion qtn_ = wamv_pose.orientation;
	        tf2::Matrix3x3 m(tf2::Quaternion(qtn_.x, qtn_.y, qtn_.z, qtn_.w));
            m.getRPY(roll_current,pitch_current,yaw_current);
        }
    }
}

/*
*描述：USV左右两轮速度发布函数
*作用：发布USV左右两轮的速度
*参数：无
*输出：无
*/
void ShuSimCore::velPublishSet(std_msgs::Float32 vel_set){
    left_vel_control.publish(vel_set);
    right_vel_control.publish(vel_set);
}

/*
*描述：USV左右两轮角度发布函数
*作用：发布USV左右两轮的角度
*参数：无
*输出：无
*/
void ShuSimCore::anglePublishSet(std_msgs::Float32 angle_set){
    left_angle_control.publish(angle_set);
    right_angle_control.publish(angle_set);
}

/*
*描述：仿真更新循环函数
*作用：更新当前状态，以及具体的仿真控制循环
*参数：无
*输出：无
*/
void ShuSimCore::tUpdate(){
    ros::Rate rate(30.0);
    while (ros::ok()) {
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose=wamv_pose;

        path_msg.poses.push_back(pose_stamped);
        model_pose.publish(path_msg);

		float target_yaw = (*usv_los_sim_ptr_)(segStart,segEnd,position_2D,usv_los_sim_ptr_->getLos());
		float pid_angle_out = (*usv_sim_pid_ptr_)(target_yaw,yaw_current,usv_sim_pid_ptr_->getAnglePid(),1);

        vel_value.data = 0.4;
        angle_value.data = pid_angle_out;

        anglePublishSet(angle_value);
        velPublishSet(vel_value);
        reference_line.publish(line_marker);
        
        ros::spinOnce();
        rate.sleep();
	}

}