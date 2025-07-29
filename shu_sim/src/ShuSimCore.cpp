#include "shu_sim/ShuSimCore.hpp"
/*
*描述：ShuSimCore类的构造函数
*作用：初始化仿真系统
*参数：无
*输出：无
*/
ShuSimCore::ShuSimCore():nh_("~"){

//初始化对象
	usv_los_sim_ptr_ = std::make_shared<TraditionalLosSim>();
    usv_sim_pid_ptr_ = std::make_shared<UsvSimPid>();

    const std::string point_parmas_file = "./src/shu_sim/params/usv_sim_path.yaml";
    usv_sim_path_ptr_ = std::make_shared<PointPath>(point_parmas_file);

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

    // 获取完整路径点
    full_path_points = usv_sim_path_ptr_->getPoints();

    // 设置路径标记
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "full_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.3; // 线宽
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    // 添加所有路径点到标记
    line_marker.points.clear();
    for (const auto& point : full_path_points) {
        geometry_msgs::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0.0;
        line_marker.points.push_back(p);
    }
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
            pose_stamped.pose=wamv_pose;
            m.getRPY(roll_current,pitch_current,yaw_current);
            if (yaw_current < 0)
            {
                yaw_current = yaw_current + 2*M_PI;
            }
            
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
        ROS_INFO("size = %.i",usv_sim_path_ptr_->getPointSize());
        for (size_t i = 0; i < usv_sim_path_ptr_->getPointSize(); i++){
            segStart = usv_sim_path_ptr_->getPoint(i);
            segEnd   = usv_sim_path_ptr_->getPoint(i+1);

            ROS_INFO("Following segment %d: (%.1f, %.1f) to (%.1f, %.1f)", 
            i, segStart[0], segStart[1], segEnd[0], segEnd[1]);

            while ( (position_2D - segEnd).norm() >= 10.0){

                float target_yaw = (*usv_los_sim_ptr_)(segStart,segEnd,position_2D,usv_los_sim_ptr_->getLos());
                float pid_angle_out = (*usv_sim_pid_ptr_)(target_yaw,yaw_current,usv_sim_pid_ptr_->getAnglePid(),1);
                
                ROS_INFO("yaw_current:%.5f",yaw_current);
                ROS_INFO("pid_angle_out:%.5f",pid_angle_out);

                vel_value.data = 0.4;
                angle_value.data = pid_angle_out;

                anglePublishSet(angle_value);
                velPublishSet(vel_value);
                reference_line.publish(line_marker);

                path_msg.poses.push_back(pose_stamped);
                model_pose.publish(path_msg);

                ros::spinOnce();
                rate.sleep();
            }
        }
	}
}