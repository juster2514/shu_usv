#include "shu_core/MixedControl.hpp"
#include "shu_core/UsvPid.hpp"
#include "shu_core/ShuControlCore.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "shu_control_core_node");

  ShuControlCore shu_control_core;

  ros::spin();
  
  return 0;
}
