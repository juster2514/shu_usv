#include "shu_core/MixedCotrol.hpp"
#include "shu_core/UsvPid.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "shu_control_core_node");

  ros::spin();
  return 0;
}
