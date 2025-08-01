#include "shu_sim/UsvSimPid.hpp"

/*
*描述：PID速度环和角度环的构造函数
*作用：初始化速度环PID和角度环PID
*参数：无
*输出：无
*/
UsvSimPid::UsvSimPid() : nh_("~") {

  //读取速度环和角度环的PID参数
  const std::string speed_pid_file = "./src/shu_sim/params/usv_sim_speed_pid.yaml";
  const std::string angle_pid_file = "./src/shu_sim/params/usv_sim_angle_pid.yaml";

  //初始化速度环和角度环PID参数
  speed_pid_ptr_ = std::make_shared<PidSimParams>(speed_pid_file);
  angle_pid_ptr_ = std::make_shared<PidSimParams>(angle_pid_file);

}

/*
*描述：计算PID输出
*作用：计算是哪个环的PID，以及用那种PID计算方法
*参数：[0]segStart:线段的起点坐标;[1]segEnd:线段的终点坐标;[2]position:当前位置坐标;[3]PID_TYPE:使用那种计算方式;
      [4]los:共享指针，指向LosParamsSim对象，包含LOS的相关参数;[5]clean_integral:是否清除积分
*输出：计算的PID值
*/
float UsvSimPid::operator()(const float target, const float current,const PidSimParams::PidType PID_TYPE,
                            std::shared_ptr<PidSimParams> pid, bool clean_integral) const {
  switch (PID_TYPE) {
    case PidSimParams::PidType::INCREMENTAL: {
      return CalculatePidIncremental(target, current, pid, clean_integral);
    }
    case PidSimParams::PidType::POSITION: {
      return CalculatePidPosition(target, current, pid, clean_integral);
    }
    default:
      break;
  }
}

/*
*描述：PID限幅函数
*作用：限制PID输出的最大值，以保护系统安全
*参数：[0]value:计算的PID输出值;[1]min:限幅的下限;[2]max:限幅的上限
*输出：限制后的PID输出
*/
float PidLimit(float value, float min, float max) {

  if (value < min) return min;
  if (value > max) return max;
  
  return value;

}

/*
*描述：位置式PID计算函数
*作用：计算PID的输出值
*参数：[0]target:期望值;[1]current:当前值;[2]pid:计算哪个环的PID;[3]clean_integral:是否清除积分
*输出：计算后的位置PID输出值
*/
const float UsvSimPid::CalculatePidPosition(float target, float current, std::shared_ptr<PidSimParams> pid, bool clean_integral) const {
  
  pid->current_ = current;
  pid->target_ = target;
  pid->error_ = pid->target_ - pid->current_;
  pid->error_integral_ += pid->error_;
  
  if (pid->use_intgral_limit_){
     pid->error_integral_ = PidLimit(pid->error_integral_, -pid->integral_limit_,pid->integral_limit_);
  }

  double p_out = pid->kp_ * pid->error_;
  double i_out = pid->ki_ * pid->error_integral_;
  double d_out = pid->kd_ * (pid->error_ - pid->last_error_);

  pid->output_ = p_out + i_out + d_out;
  pid->last_error_ = pid->error_;

  if (pid->use_output_limit_) pid->output_ = PidLimit(pid->output_, -pid->output_limit_, pid->output_limit_);
  
  /*调试使用*/
  // ROS_INFO_STREAM("PID_name: " << pid->pid_name_
  //               <<"pid->output_:"<<pid->output_);

  return pid->output_;

}

/*
*描述：增量式PID计算函数
*作用：计算PID的输出值
*参数：[0]target:期望值;[1]current:当前值;[2]pid:计算哪个环的PID;[3]clean_integral:是否清除积分
*输出：计算后的增量式PID输出值
*/
const float UsvSimPid::CalculatePidIncremental(float target, float current, std::shared_ptr<PidSimParams> pid, bool clean_integral) const {
  
  pid->current_ = current;

  pid->target_ = target;

  pid->error_ = pid->target_ - pid->current_;


  if (clean_integral) {
      pid->last_error_ = 0.0;
      pid->last_last_error_ = 0.0;
      pid->last_output_ = 0.0;
  }

  double p_out = pid->kp_ * (pid->error_ - pid->last_error_);
  double i_out = pid->ki_ * pid->error_;
  double d_out = pid->kd_ * (pid->error_ - 2 * pid->last_error_ + pid->last_last_error_);

  pid->output_ = pid->last_output_ + p_out + i_out + d_out;

  if (pid->use_output_limit_) {
      pid->output_ = PidLimit(pid->output_, -pid->output_limit_, pid->output_limit_);
  }

  pid->last_last_error_ = pid->last_error_; 
  pid->last_error_ = pid->error_;
  pid->last_output_ = pid->output_;          

  /*调试使用*/
  // ROS_INFO_STREAM("PID_name: " << pid->pid_name_ 
  //                << " output: " << pid->output_);
                  
  return pid->output_;

}
