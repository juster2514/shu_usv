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
    file["delta"] >> delta_;
    file["max_heading_rate"] >> max_heading_rate_;
  };

  ~LosParams() = default;
  
  Eigen::Vector2d position_;
  Eigen::Vector2d segStart_;
  Eigen::Vector2d segEnd_;
  Eigen::Vector2d segVector_;
  Eigen::Vector2d pointToStart_;
  Eigen::Vector2d projection_;
  Eigen::Vector2d closestPoint_;
  
  double delta_{0.0}, max_heading_rate_{0.0} ;
  double gamma_ ;
  double y_e_ ;
  double chi_;

};

class TraditionalLOS
{
 public:
  explicit TraditionalLOS();
  ~TraditionalLOS() = default;

  void updatePosition(Eigen::Vector2d position,std::shared_ptr<LosParams> los) const;
  void getClosestPoint(Eigen::Vector2d segStart , Eigen::Vector2d segEnd , std::shared_ptr<LosParams> los) const;

  const float CalculateLos(std::shared_ptr<LosParams> los ) const;

  float operator()(Eigen::Vector2d segStart, Eigen::Vector2d segEnd, Eigen::Vector2d position , std::shared_ptr<LosParams> los) const;

 private:

  ros::NodeHandle nh_;

  std::shared_ptr<LosParams> traditional_los_ptr_;


};


#endif
