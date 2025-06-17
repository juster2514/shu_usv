#ifndef SHUCONTROLCORE_HPP
#define SHUCONTROLCORE_HPP

#include "shu_core/BottomMessageSub.hpp"
#include "shu_core/MixedCotrol.hpp"
#include "shu_core/UsvPid.hpp"

class ShuControlCore{
 public:

    ShuControlCore();
    ~ShuControlCore() = default;

 private:
 
    ros::NodeHandle nh_;

};

#endif