#ifndef GLOBAL_ROBOT_LOCALIZATION__SIM_NODE_HPP_
#define GLOBAL_ROBOT_LOCALIZATION__SIM_NODE_HPP_

#include <memory>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace global_robot_localization
{

class SimNode : public rclcpp::Node
{
public:
  explicit SimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct RuntimeOptions;

  RuntimeOptions readRuntimeOptions() const;
  void publishStaticMap();
  void publishSimulationStep();

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid map_msg_;
};

}  // namespace global_robot_localization

#endif  // GLOBAL_ROBOT_LOCALIZATION__SIM_NODE_HPP_
