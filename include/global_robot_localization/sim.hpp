#ifndef GLOBAL_ROBOT_LOCALIZATION__SIM_HPP_
#define GLOBAL_ROBOT_LOCALIZATION__SIM_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "global_robot_localization/localization_search.hpp"

namespace global_robot_localization
{

struct GeneratedMap
{
  nav_msgs::msg::OccupancyGrid msg;
};

struct SimulatedLidar
{
  LaserScanPoints endpoints_base;
  sensor_msgs::msg::LaserScan scan;
};

GeneratedMap makeEmptyMap(std::uint32_t width, std::uint32_t height, double resolution);
GeneratedMap makeAsymmetricMap();
GeneratedMap makePathologicallySymmetricMap();

SimulatedLidar simulateLidar(
  const nav_msgs::msg::OccupancyGrid & map,
  const Pose2D & robot_pose,
  SearchOptions & options,
  int beam_count = 91,
  double field_of_view = kTwoPi,
  double max_range = 5.5);

std::vector<Eigen::Vector2d> transformEndpointsToMap(
  const std::vector<Eigen::Vector2d> & endpoints_base,
  const Pose2D & pose);

}  // namespace global_robot_localization

#endif  // GLOBAL_ROBOT_LOCALIZATION__SIM_HPP_
