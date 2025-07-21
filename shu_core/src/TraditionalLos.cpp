#include "shu_core/TraditionalLos.hpp"

/*
*描述：TraditionalLOS类的构造函数
*作用：初始化ROS节点句柄，并加载LOS参数文件，创建LosParams对象的共享指针
*参数：无
*输出：无
*/
TraditionalLOS::TraditionalLOS() : nh_("~") {

  const std::string los_parmas_file = "./src/shu_core/params/usv_speed_pid.yaml";

  traditional_los_ptr_ = std::make_shared<LosParams>(los_parmas_file);

}

/*
*描述：计算LOS（视线）的输出值
*作用：更新LOS位置，获取线段上最近的点，并计算LOS的输出值
*参数：[0]segStart:线段的起点坐标;[1]segEnd:线段的终点坐标;[2]position:当前位置坐标;[3]los:共享指针，指向LosParams对象，包含LOS的相关参数
*输出：LOS的输出值，类型为float
*/
float TraditionalLOS::operator()(Eigen::Vector2d segStart, Eigen::Vector2d segEnd, Eigen::Vector2d position ,std::shared_ptr<LosParams> los) const {

    updatePosition(position,los);
    
    getClosestPoint(segStart,segEnd,los);

    return CalculateLos(los);

}

/*
*描述：更新LOS（视线）的位置
*作用：将给定的位置更新到LOS参数中
*参数：[0]position:新的位置坐标;[1]los:共享指针，指向LosParams对象，包含LOS的相关参数
*输出：无
*/
void TraditionalLOS::updatePosition(Eigen::Vector2d position,std::shared_ptr<LosParams> los) const{

    los->position_ = position;

}

/*
*描述：获取线段上距离LOS位置最近的点
*作用：计算并更新LOS参数中存储的最近点
*参数：[0]segStart:线段的起点坐标;[1]segEnd:线段的终点坐标;[2]los:共享指针，指向LosParams对象，包含LOS的相关参数
*输出：无
*/
void TraditionalLOS::getClosestPoint(Eigen::Vector2d segStart , Eigen::Vector2d segEnd , std::shared_ptr<LosParams> los) const {

    los->segVector_ = segEnd - segStart;

    los->pointToStart_ = los->position_ - segStart;

    double segLength = sqrt(los->segVector_.squaredNorm());

    double proj = los->pointToStart_.dot(los->segVector_) / segLength;

    if (proj < 0.0) { proj = 0.0; } 
    else if (proj > segLength) { proj = segLength; }

    los->closestPoint_ = segStart + (proj * los->segVector_ / segLength);

}

/*
*描述：计算LOS（视线）的输出角度
*作用：根据LOS参数计算当前LOS的输出角度
*参数：[0]los:共享指针，指向LosParams对象，包含LOS的相关参数
*输出：LOS的输出角度
*/
const float TraditionalLOS::CalculateLos(std::shared_ptr<LosParams> los ) const{

    los->gamma_ =  atan2(los->segVector_[1],los->segVector_[0]);// y在前，x在后

    los->y_e_ =  (los->closestPoint_[0] - los->position_[0]) * sin(los->gamma_) + 
                 (los->closestPoint_[1] - los->position_[1]) * cos(los->gamma_);

    los->chi_ = los->gamma_ - atan2(los->y_e_,los->delta_);

    return los->chi_;
}

