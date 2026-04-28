#include "global_robot_localization/sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "global_robot_localization/localization_search.hpp"
#include "global_robot_localization/sim.hpp"

namespace global_robot_localization
{

struct SimNode::RuntimeOptions
{
  std::string map_frame{"map"};
  std::string scan_frame{"base_link"};
  double scan_rate_hz{5.0};
  double robot_x{2.45};
  double robot_y{2.25};
  double robot_yaw{0.65};
  int beam_count{181};
  double field_of_view{ kTwoPi };
  double max_range{6.0};
  double sigma_r{0.3};
  double sigma_theta{0.005};
  double alpha_m{1.0};
};

SimNode::SimNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sim", options)
{
  declare_parameter("map_frame", "map");
  declare_parameter("scan_frame", "base_link");
  declare_parameter("scan_rate_hz", 5.0);
  declare_parameter("robot_x", 2.45);
  declare_parameter("robot_y", 2.25);
  declare_parameter("robot_yaw", 0.65);
  declare_parameter("beam_count", 91);
  declare_parameter("field_of_view", kTwoPi);
  declare_parameter("max_range", 5.5);
  declare_parameter("sigma_r", 0.03);
  declare_parameter("sigma_theta", 0.005);
  declare_parameter("alpha_m", 1.0);

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable());
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS());

  map_msg_ = makeAsymmetricMap().msg;
  publishStaticMap();

  const auto runtime = readRuntimeOptions();
  const auto period = std::chrono::duration<double>(1.0 / std::max(runtime.scan_rate_hz, 1.0e-3));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SimNode::publishSimulationStep, this));
}

SimNode::RuntimeOptions SimNode::readRuntimeOptions() const
{
  RuntimeOptions options;
  options.map_frame = get_parameter("map_frame").as_string();
  options.scan_frame = get_parameter("scan_frame").as_string();
  options.scan_rate_hz = get_parameter("scan_rate_hz").as_double();
  options.robot_x = get_parameter("robot_x").as_double();
  options.robot_y = get_parameter("robot_y").as_double();
  options.robot_yaw = get_parameter("robot_yaw").as_double();
  options.beam_count = static_cast<int>(get_parameter("beam_count").as_int());
  options.field_of_view = get_parameter("field_of_view").as_double();
  options.max_range = get_parameter("max_range").as_double();
  options.sigma_r = get_parameter("sigma_r").as_double();
  options.sigma_theta = get_parameter("sigma_theta").as_double();
  options.alpha_m = get_parameter("alpha_m").as_double();

  options.scan_rate_hz = std::max(options.scan_rate_hz, 1.0e-3);
  options.beam_count = std::max(options.beam_count, 2);
  return options;
}

void SimNode::publishStaticMap()
{
  const auto runtime = readRuntimeOptions();
  map_msg_.header.stamp = now();
  map_msg_.header.frame_id = runtime.map_frame;
  map_pub_->publish(map_msg_);
}

void SimNode::publishSimulationStep()
{
  const auto runtime = readRuntimeOptions();
  const auto stamp = now();

  SearchOptions search_options;
  search_options.sigma_r = runtime.sigma_r;
  search_options.sigma_theta = runtime.sigma_theta;
  search_options.alpha_m = runtime.alpha_m;

  const Pose2D robot_pose{runtime.robot_x, runtime.robot_y, runtime.robot_yaw};
  auto simulated = simulateLidar(
    map_msg_,
    robot_pose,
    search_options,
    runtime.beam_count,
    runtime.field_of_view,
    runtime.max_range);

  simulated.scan.header.stamp = stamp;
  simulated.scan.header.frame_id = runtime.scan_frame;
  simulated.scan.scan_time = static_cast<float>(1.0 / runtime.scan_rate_hz);
  if (runtime.beam_count > 1) {
    simulated.scan.time_increment = simulated.scan.scan_time /
      static_cast<float>(runtime.beam_count - 1);
  }

  scan_pub_->publish(simulated.scan);
}

}  // namespace global_robot_localization
