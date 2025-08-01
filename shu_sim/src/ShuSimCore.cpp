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
    usv_left_vel = nh_.subscribe<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd",10,&ShuSimCore::modelLeftVelCallback,this);
    usv_right_vel = nh_.subscribe<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd",10,&ShuSimCore::modelRightVelCallback,this);

//初始化可视化参数
    shuSimVisualizationInit();

//仿真更新循环
    tUpdate();
}

/*
*描述：可视化参数初始化函数
*作用：设置一些可视化参数
*参数：无
*输出：无
*/
void ShuSimCore::shuSimVisualizationInit(){

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
void ShuSimCore::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    // 遍历所有模型名称
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "wamv") {  // 检查是否为wamv模型
        
            // 提取位置和姿态
            wamv_pose = msg->pose[i];

            wamv_twist = msg->twist[i];

	        position_2D[0] = wamv_pose.position.x;
	        position_2D[1] = wamv_pose.position.y;

            //将四元数转为欧拉角
            geometry_msgs::Quaternion qtn_ = wamv_pose.orientation;
	        tf2::Matrix3x3 m(tf2::Quaternion(qtn_.x, qtn_.y, qtn_.z, qtn_.w));
            pose_stamped.pose=wamv_pose;
            m.getRPY(roll_current,pitch_current,yaw_current);

            //进行归一化处理,从(-π~π)映射到(0~2π)
            if (yaw_current < 0)
            {
                yaw_current = yaw_current + 2*M_PI;
            }
            
        }
    }
}

/*
*描述：USV左轮速度订阅回调函数
*作用：订阅USV左轮的速度
*参数：无
*输出：无
*/
void ShuSimCore::modelLeftVelCallback(const std_msgs::Float32::ConstPtr& msg){
    left_speed_current = msg->data;
}

/*
*描述：USV右两轮速度订阅回调函数
*作用：订阅USV右轮的速度
*参数：无
*输出：无
*/
void ShuSimCore::modelRightVelCallback(const std_msgs::Float32::ConstPtr& msg){
    right_speed_current = msg->data;
}

/*
*描述：USV左右两轮速度发布函数
*作用：发布USV左右两轮的速度
*参数：[0]vel_set:设置的速度值
*输出：无
*/
void ShuSimCore::velPublishSet(std_msgs::Float32 vel_set){
    left_vel_control.publish(vel_set);
    right_vel_control.publish(vel_set);
}

/*
*描述：USV左右两轮角度发布函数
*作用：发布USV左右两轮的角度
*参数：[0]angle_set:设置的角度值
*输出：无
*/
void ShuSimCore::anglePublishSet(std_msgs::Float32 angle_set){
    left_angle_control.publish(angle_set);
    right_angle_control.publish(angle_set);
}

/*
*描述：计算USV旋转的最小角度函数
*作用：计算当前位置与期望位置的最小转动角度，作为增量式PID控制的期望输入
*参数：[0]current:当前角度；[1]target:目标角度
*输出：归一化后的角度增量，范围为(-π~π)
*/
float ShuSimCore::calculateAngleError(float current, float target) {
    float error = target - current;
    error = fmod(error + M_PI, 2 * M_PI);
    return (error < 0) ? error + M_PI : error - M_PI;
}

/*
*描述：仿真更新循环函数
*作用：更新当前状态，以及具体的仿真控制循环
*参数：无
*输出：无
*/
void ShuSimCore::tUpdate(){

    ros::Rate rate(30.0); //设置仿真频率

    /*变量实际意义为增量式当前已经转的角度，该变量正常增量式PID需每次反馈后进行累加，
    但本控制系统的增量角度一直在更新并减小，所以该变量应为常量0*/
    float delta_current = 0.0f;

    while (ros::ok()) {

        ROS_INFO("size = %.i",usv_sim_path_ptr_->getPointSize());

        for (size_t i = 0; i < usv_sim_path_ptr_->getPointSize(); i++){

            segStart = usv_sim_path_ptr_->getPoint(i);
            segEnd   = usv_sim_path_ptr_->getPoint(i+1);

            ROS_INFO("Following segment %ld: (%.1f, %.1f) to (%.1f, %.1f)", 
                    i, segStart[0], segStart[1], segEnd[0], segEnd[1]);

            //当距离当前跟踪路线终点距离小于10m时切换下一段路线
            while ( (position_2D - segEnd).norm() >= 10.0){

                los_result = (*usv_los_sim_ptr_)(segStart,segEnd,position_2D,usv_los_sim_ptr_->getLos());
                
                float target_yaw = calculateAngleError(yaw_current,los_result.angle);
                float target_speed = los_result.speed;

                //刚开始运行时，角度和速度有可能出现无效数字，需要去除其无效数字
                if (std::isnan(target_yaw)) {
                    ROS_ERROR("Invalid target_yaw!");
                    target_yaw = 0.0;
                }
                
                if (std::isnan(target_speed)) {
                    ROS_ERROR("Invalid target_yaw!");
                    target_speed = 0.0;
                }
                
                float pid_angle_out = (*usv_sim_pid_ptr_)(target_yaw,delta_current,PidSimParams::INCREMENTAL,usv_sim_pid_ptr_->getAnglePid(),1);

                float pid_speed_out = (*usv_sim_pid_ptr_)(target_speed,left_speed_current,PidSimParams::POSITION,usv_sim_pid_ptr_->getSpeedPid(),1);


                vel_value.data = pid_speed_out;
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