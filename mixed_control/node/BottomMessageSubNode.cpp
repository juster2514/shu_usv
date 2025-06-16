#include "mixed_control/BottomMessageSub.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bottom_message_sub_node");
  BottomMessageSub bottom_message_sub;
  ros::spin();
  return 0;
}