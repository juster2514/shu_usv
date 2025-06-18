#ifndef MIXEDCONTROL_HPP
#define MIXEDCONTROL_HPP

#include <ros/ros.h>
#include <memory>
#include <thread>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ActuatorControl.h>

class MixedControl{
 public:
    MixedControl();
    ~MixedControl() = default;

    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void actuator_set(float yaw , float thrust);
    void check_arm();

    mavros_msgs::SetMode getSetMode() const { return offb_set_mode; };
    mavros_msgs::CommandBool getArm() const { return arm_cmd; };
    mavros_msgs::State getCurrentState() const { return current_state; };
    mavros_msgs::ActuatorControl getActuator() const { return actuator_control; };

 private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State current_state;
    mavros_msgs::ActuatorControl actuator_control;

};

#endif