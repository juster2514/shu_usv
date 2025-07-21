#include "shu_core/MixedControl.hpp"

/*
*描述：USV混控对象的构造函数
*作用：初始化USV混控
*参数：无
*输出：无
*/
MixedControl::MixedControl():nh_("~"){

    //初始化发布话题
    ros::Publisher local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    //初始化订阅话题
    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("/mavros/state",10,&MixedControl::state_callback,this);
    ros::Subscriber rc_sub = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",10,&MixedControl::rc_callback,this);

    //初始化服务
    ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //初始位置点
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //初始先向pixhawk发布一些点
    for(int i = 200; ros::ok() && i > 0; --i){
        ros::Rate rate(20.0);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //设置OFFBOARD模式和解锁
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    ROS_INFO("run mixcontrol");

    check_arm();


}

/*
*描述：USV混控的遥控器回调函数
*作用：获取遥控器的值
*参数：无
*输出：无
*/
void MixedControl::rc_callback(const mavros_msgs::RCIn::ConstPtr& msg){
	rc_in = *msg;
}

/*
*描述：pixhawk状态的回调函数
*作用：获取pixhawk的状态，如当前模式、是否解锁
*参数：无
*输出：无
*/
void MixedControl::state_callback(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

/*
*描述：混控的输出函数
*作用：输出指定的混控值，以达到控制底层电机和舵机
*参数：[0]yaw:舵机角度；[1]thrust:油门大小
*输出：无
*备注：其中混控的两个值范围为[-1,1],其中-1为油门最大和舵机最左。
     （后续考虑是否进行映射为油门范围为0-100,舵机角度为-10°-10°）
*/
void MixedControl::actuator_set(float yaw , float thrust){
    actuator_control.group_mix = 0;
    actuator_control.controls[2]=yaw;
    actuator_control.controls[3]=thrust;
}

/*
*描述：pixhawk检查和解锁函数
*作用：检查pixhawk是否解锁以及是否为OFFBOARD模式
*参数：无
*输出：无
*/
void MixedControl::check_arm(){

    ros::Rate rate(20.0);//设置频率为20HZ
    ros::Time last_request = ros::Time::now();

    //等待pixhawk连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("not connected");
    }

    while(ros::ok()){

        //当想切换手动模式时，需切换遥控器左上的开关，并保证右上的开关处于解锁状态
        if(rc_in.channels[4] == 1999){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            local_pos_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

}