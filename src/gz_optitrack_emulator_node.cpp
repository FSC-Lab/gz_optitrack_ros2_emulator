#include <vector>
#include <array>
#include <string>

#include "gz_optitrack_emulator.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<GazeboMocapEmulator>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}