#include "isr_m3_driver/isr_m3_driver.hpp"

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto isr_m3 = std::make_shared<ISR_M3>();
  rclcpp::spin(isr_m3);
  rclcpp::shutdown();
  return 0;
}
