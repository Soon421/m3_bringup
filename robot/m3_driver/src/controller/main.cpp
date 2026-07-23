#include "manual_controller/manual_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto manual_controller = std::make_shared<ManualController>();
  rclcpp::spin(manual_controller);
  rclcpp::shutdown();
  return 0;
}