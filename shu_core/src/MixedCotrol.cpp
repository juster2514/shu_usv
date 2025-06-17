#include "shu_core/MixedCotrol.hpp"


MixedControl::MixedControl():nh_("~"){
    
    ros::Publisher actuator_control_pub = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control",10);

    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("/mavros/state",10,&MixedControl::state_callback,this);

    ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    check_arm();

}


void MixedControl::state_callback(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void MixedControl::actuator_set(float yaw , float thrust){
    actuator_control.group_mix = 0;
    actuator_control.controls[2]=yaw;
    actuator_control.controls[3]=thrust;
}

void MixedControl::check_arm(){

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    while (current_state.mode != "OFFBOARD" || !current_state.armed )
    {
        if( current_state.mode != "OFFBOARD" ){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
        } 

        if( !current_state.armed ){
            if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
        }

        ros::spinOnce();
        rate.sleep();

    }

}