#ifndef USVSIMPID_HPP
#define USVSIMPID_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

struct PidSimParams {
  PidSimParams(float kp, float ki, float kd)
      : kp_(kp), ki_(ki), kd_(kd), pid_name_(""){};
  PidSimParams(const std::string& pid_parmas_file) {
    cv::FileStorage file(pid_parmas_file, cv::FileStorage::READ);
    file["Kp"] >> kp_;
    file["Ki"] >> ki_;
    file["Kd"] >> kd_;
    file["Pid.Name"] >> pid_name_;
    file["Output.Limit"] >> output_limit_;
    file["Integal.Limit"] >> integral_limit_;
    file["Use.Integal.Limit"] >> use_intgral_limit_;
    file["Use.Output.Limit"] >> use_output_limit_;
    file["CalculateTime"] >> calculate_time_;
  };
  enum PidType { INCREMENTAL, POSITION };
  ~PidSimParams() = default;
  std::string pid_name_;
  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double last_error_{0.0}, error_{0.0}, error_integral_{0.0};
  double last_output_{0.0}, last_last_error_{0.0};
  double target_{0.0}, current_{0.0};
  double output_{0.0};
  double pid_error_integral_limit_{0.0};
  double output_limit_{0.0}, integral_limit_{0.0};
  bool use_intgral_limit_{false};
  bool use_output_limit_{true};
  int  calculate_time_{10};

};

class UsvSimPid
{
 public:
  explicit UsvSimPid();
  ~UsvSimPid() = default;

  float operator()(const float target, const float current,
                   const PidSimParams::PidType PID_TYPE,
                   std::shared_ptr<PidSimParams> pid,bool clean_integral) const;

  const float CalculatePidPosition(float target, float current,std::shared_ptr<PidSimParams> pid,bool clean_integral) const;
  const float CalculatePidIncremental(float target, float current,std::shared_ptr<PidSimParams> pid,bool clean_integral) const;

  std::shared_ptr<PidSimParams> getAnglePid() const { return angle_pid_ptr_; };
  std::shared_ptr<PidSimParams> getSpeedPid() const { return speed_pid_ptr_; };

 private:

  ros::NodeHandle nh_;
  std::shared_ptr<PidSimParams> angle_pid_ptr_;
  std::shared_ptr<PidSimParams> speed_pid_ptr_;

};


#endif
