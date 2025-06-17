#include "shu_core/MixedCotrol.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mixed_control_node");
  MixedControl mixed_control;
  ros::spin();
  return 0;
}