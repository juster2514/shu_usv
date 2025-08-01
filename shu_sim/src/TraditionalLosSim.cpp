#include "shu_sim/TraditionalLosSim.hpp"

/*
*描述：TraditionalLosSim类的构造函数
*作用：初始化ROS节点句柄，并加载LOS参数文件，创建LosParamsSim对象的共享指针
*参数：无
*输出：无
*/
TraditionalLosSim::TraditionalLosSim() : nh_("~") {

  const std::string los_parmas_file = "./src/shu_sim/params/usv_traditional_los_sim.yaml";

  traditional_los_ptr_sim_ = std::make_shared<LosParamsSim>(los_parmas_file);

}

/*
*描述：计算LOS（视线）的输出值
*作用：更新LOS位置，获取线段上最近的点，并计算LOS的输出值
*参数：[0]segStart:线段的起点坐标;[1]segEnd:线段的终点坐标;[2]position:当前位置坐标;[3]los:共享指针，指向LosParamsSim对象，包含LOS的相关参数
*输出：LOS的输出值，类型为float
*/
LosSimOut TraditionalLosSim::operator()(Eigen::Vector2d segStart, Eigen::Vector2d segEnd, Eigen::Vector2d position ,std::shared_ptr<LosParamsSim> los) const {

    updatePosition(position,los);
    
    getClosestPoint(segStart,segEnd,los);

    return {CalculateLosAngle(los),CalculateLosSpeed(los)};

}

/*
*描述：更新LOS（视线）的位置
*作用：将给定的位置更新到LOS参数中
*参数：[0]position:新的位置坐标;[1]los:共享指针，指向LosParamsSim对象，包含LOS的相关参数
*输出：无
*/
void TraditionalLosSim::updatePosition(Eigen::Vector2d position,std::shared_ptr<LosParamsSim> los) const{

    los->position_ = position;

}

/*
*描述：获取线段上距离LOS位置最近的点
*作用：计算并更新LOS参数中存储的最近点
*参数：[0]segStart:线段的起点坐标;[1]segEnd:线段的终点坐标;[2]los:共享指针，指向LosParamsSim对象，包含LOS的相关参数
*输出：无
*/
void TraditionalLosSim::getClosestPoint(Eigen::Vector2d segStart , Eigen::Vector2d segEnd , std::shared_ptr<LosParamsSim> los) const {

    los->segVector_ = segEnd - segStart;

    los->pointToStart_ = los->position_ - segStart;

    los->cross_ = los->segVector_.x() * los->pointToStart_.y() - los->segVector_.y() * los->pointToStart_.x();

    double segLength = sqrt(los->segVector_.squaredNorm());

    double proj = los->pointToStart_.dot(los->segVector_) / segLength;

    if (proj < 0.0) { proj = 0.0; } 
    else if (proj > segLength) { proj = segLength; }

    los->closestPoint_ = segStart + (proj * los->segVector_ / segLength);

}

/*
*描述：计算LOS（视线）的输出弧度
*作用：根据LOS参数计算当前LOS的输出弧度
*参数：[0]los:共享指针，指向LosParams对象，包含LOS的相关参数
*输出：los的输出弧度
*/
const float TraditionalLosSim::CalculateLosAngle(std::shared_ptr<LosParamsSim> los ) const{

    los->gamma_ = atan2(los->segVector_[1], los->segVector_[0]);

    const float dx = los->closestPoint_[0] - los->position_[0];
    const float dy = los->closestPoint_[1] - los->position_[1];
    const float sin_gamma = sin(los->gamma_);
    const float cos_gamma = cos(los->gamma_);
    los->y_e_ = fabs(dx) * fabs(sin_gamma) + fabs(dy) * fabs(cos_gamma);

    const float atan_val = atan2(los->y_e_, los->delta_);
    
    const bool is_first_quadrant = fabs(los->gamma_) < M_PI / 2;
    if (is_first_quadrant) {
        if (los->cross_ <= 0.0) {
            los->chi_ = los->gamma_ + atan_val;
        } else {
            los->chi_ = los->gamma_ - atan_val;
        }
    } else {
        if (los->cross_ > 0.0) {
            los->chi_ = los->gamma_ - atan_val;
        } else {
            los->chi_ = los->gamma_ + atan_val;
        }
    }

    // 角度归一化到[0, 2π)
    if (los->chi_ < 0) {
        los->chi_ += 2 * M_PI;
    }

    return los->chi_;
}


const float TraditionalLosSim::CalculateLosSpeed(std::shared_ptr<LosParamsSim> los ) const{
        
        // 核心计算公式
    double reduction_factor = los->k_ * tanh(los->y_e_ / los->beta_);
        
        // 计算并限制速度范围
    double desired_speed = los->max_speed_ * (1.0 - reduction_factor);
    double min_speed = los->max_speed_ * (1.0 - los->k_) * 0.5;

    if (desired_speed < min_speed) desired_speed = min_speed;
    if (desired_speed > los->max_speed_) desired_speed = los->max_speed_;

    return desired_speed;
}

        
