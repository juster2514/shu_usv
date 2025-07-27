#include "shu_sim/ShuSimCore.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "shu_sim_node");

  ShuSimCore shu_sim_core;

  ros::spin();
  
  return 0;
}