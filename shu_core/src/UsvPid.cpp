#include "shu_core/UsvPid.hpp"

UsvPid::UsvPid() : nh_("~") {

  const std::string speed_pid_file = "./src/shu_core/params/usv_speed_pid.yaml";
  const std::string angle_pid_file = "./src/shu_core/params/usv_angle_pid.yaml";

  speed_pid_ptr_ = std::make_shared<PidParams>(speed_pid_file);
  angle_pid_ptr_ = std::make_shared<PidParams>(angle_pid_file);

}

float UsvPid::operator()(const float target, const float current,std::shared_ptr<PidParams> pid, bool clean_integral) const {

    return CalculatePid(target, current, pid, clean_integral);

}

float PidLimit(float value, float min, float max) {

  if (value < min) return min;
  if (value > max) return max;
  
  return value;

}

const float UsvPid::CalculatePid(float target, float current, std::shared_ptr<PidParams> pid, bool clean_integral) const {

  pid->current_ = current;
  pid->target_ = target;
  pid->error_ = pid->target_ - pid->current_;
  pid->error_integral_ += pid->error_;
  
  if (pid->use_intgral_limit_)
    pid->error_integral_ = PidLimit(pid->error_integral_, -pid->integral_limit_,
                                    pid->integral_limit_);

  double p_out = pid->kp_ * pid->error_;
  double i_out = pid->ki_ * pid->error_integral_;
  double d_out = pid->kd_ * (pid->error_ - pid->last_error_);

  pid->output_ = p_out + i_out + d_out;
  pid->last_error_ = pid->error_;

  if (pid->use_output_limit_) pid->output_ = PidLimit(pid->output_, -pid->output_limit_, pid->output_limit_);

  return pid->output_;

}
