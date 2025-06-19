#include "shu_core/TraditionalLos.hpp"

TraditionalLOS::TraditionalLOS() : nh_("~") {

  const std::string los_parmas_file = "./src/shu_core/params/usv_speed_pid.yaml";

  traditional_los_ptr_ = std::make_shared<LosParams>(los_parmas_file);

}

float TraditionalLOS::operator()(Eigen::Vector2d segStart, Eigen::Vector2d segEnd, Eigen::Vector2d position ,std::shared_ptr<LosParams> los) const {

    updatePosition(position,los);
    
    getClosestPoint(segStart,segEnd,los);

    return CalculateLos(los);

}


void TraditionalLOS::updatePosition(Eigen::Vector2d position,std::shared_ptr<LosParams> los) const{

    los->position_ = position;

}



void TraditionalLOS::getClosestPoint(Eigen::Vector2d segStart , Eigen::Vector2d segEnd , std::shared_ptr<LosParams> los) const {

    los->segVector_ = segEnd - segStart;

    los->pointToStart_ = los->position_ - segStart;

    double segLength = sqrt(los->segVector_.squaredNorm());

    double proj = los->pointToStart_.dot(los->segVector_) / segLength;

    if (proj < 0.0) { proj = 0.0; } 
    else if (proj > segLength) { proj = segLength; }

    los->closestPoint_ = segStart + (proj * los->segVector_ / segLength);

}

const float TraditionalLOS::CalculateLos(std::shared_ptr<LosParams> los ) const{

    los->gamma_ =  atan2(los->segVector_[0],los->segVector_[1]);

    los->y_e_ =  (los->closestPoint_[0] - los->position_[0]) * sin(los->gamma_) + 
                 (los->closestPoint_[1] - los->position_[1]) * cos(los->gamma_);

    los->chi_ = los->gamma_ - atan2(los->y_e_,los->delta_);

    return los->chi_;
}

