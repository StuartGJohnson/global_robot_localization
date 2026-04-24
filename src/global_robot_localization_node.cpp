#include <rclcpp/rclcpp.hpp>

#include "global_robot_localization/global_robot_localization.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_robot_localization::GlobalRobotLocalization>());
  rclcpp::shutdown();
  return 0;
}
