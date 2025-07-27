#ifndef TRADITIONALLOS_HPP
#define TRADITIONALLOS_HPP

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>

struct LosParams {
  LosParams(float delta, float max_heading_rate ): delta_(delta), max_heading_rate_(max_heading_rate) {};
  LosParams(const std::string& los_parmas_file) {
    cv::FileStorage file(los_parmas_file, cv::FileStorage::READ);
    file["usv_length"] >> usv_length_;
    file["usv_length"] >> lammda_;
    file["max_heading_rate"] >> max_heading_rate_;
    delta_ = usv_length_ * lammda_;
  };

  ~LosParams() = default;
  
  Eigen::Vector2d position_;
  Eigen::Vector2d segStart_;
  Eigen::Vector2d segEnd_;
  Eigen::Vector2d segVector_;
  Eigen::Vector2d pointToStart_;
  Eigen::Vector2d projection_;
  Eigen::Vector2d closestPoint_;
  
  double usv_length_;
  double lammda_;
  double delta_;
  double max_heading_rate_ ;
  
  double gamma_ ;
  double y_e_ ;
  double chi_;

};

class TraditionalLos
{
 public:
  explicit TraditionalLos();
  ~TraditionalLos() = default;

  float operator()(Eigen::Vector2d segStart, Eigen::Vector2d segEnd, Eigen::Vector2d position , std::shared_ptr<LosParams> los) const;

  void updatePosition(Eigen::Vector2d position,std::shared_ptr<LosParams> los) const;
  void getClosestPoint(Eigen::Vector2d segStart , Eigen::Vector2d segEnd , std::shared_ptr<LosParams> los) const;

  const float CalculateLos(std::shared_ptr<LosParams> los ) const;

  std::shared_ptr<LosParams> getLos() const { return traditional_los_ptr_; };


 private:

  ros::NodeHandle nh_;

  std::shared_ptr<LosParams> traditional_los_ptr_;


};


#endif
