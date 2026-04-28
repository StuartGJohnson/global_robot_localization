#include "global_robot_localization/global_robot_localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

namespace global_robot_localization
{

namespace
{

double secondsFromDuration(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool isZeroDuration(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec == 0 && duration.nanosec == 0U;
}

}  // namespace

struct GlobalRobotLocalization::RuntimeOptions
{
  int occupied_threshold{50};
  bool unknown_is_occupied{true};
  std::string base_frame{"base_link"};
  int num_scans{1};
  double scan_collection_timeout_sec{3.0};
  double transform_timeout_sec{0.2};
  SearchOptions search;
  bool publish_initialpose{true};
  bool publish_markers{true};
};

GlobalRobotLocalization::GlobalRobotLocalization(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_robot_localization", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_, this, true)
{
  declare_parameter("occupied_threshold", 50);
  declare_parameter("unknown_is_occupied", true);
  declare_parameter("base_frame", "base_link");
  declare_parameter("num_scans", 1);
  declare_parameter("max_scan_buffer_size", 100);
  declare_parameter("scan_collection_timeout_sec", 3.0);
  declare_parameter("transform_timeout_sec", 0.2);
  declare_parameter("coarse_xy_step", 0.1);
  declare_parameter("coarse_yaw_step_deg", 5.0);
  declare_parameter("top_k", 40);
  declare_parameter("candidate_min_xy_separation", 0.45);
  declare_parameter("candidate_min_yaw_separation_deg", 8.0);
  declare_parameter("refine_levels", 4);
  declare_parameter("refine_xy_step", 0.15);
  declare_parameter("refine_yaw_step_deg", 4.0);
  declare_parameter("scan_stride", 1);
  declare_parameter("max_range", 5.5);
  declare_parameter("off_map_distance", 3.0);
  declare_parameter("sigma_r", 0.03);
  declare_parameter("sigma_theta", 0.005);
  declare_parameter("alpha_m", 1.0);
  declare_parameter("map_padding_xy", 1.0);
  declare_parameter("free_space_weight", 0.4);
  declare_parameter("free_space_sample_step", 0.2);
  declare_parameter("min_endpoint_count", 80);
  declare_parameter("covariance_step_xy", 0.03);
  declare_parameter("covariance_step_yaw_deg", 1.0);
  declare_parameter("covariance_regularization", 1.0e-3);
  declare_parameter("covariance_scale", 1.0);
  declare_parameter("publish_initialpose", true);
  declare_parameter("publish_markers", true);

  max_scan_buffer_size_ = static_cast<std::size_t>(
    std::max<std::int64_t>(1, get_parameter("max_scan_buffer_size").as_int()));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&GlobalRobotLocalization::mapCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS(),
    std::bind(&GlobalRobotLocalization::scanCallback, this, std::placeholders::_1));

  initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(1));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "candidate_poses", rclcpp::QoS(1));

  action_server_ = rclcpp_action::create_server<LocalizeInMap>(
    this,
    "LocalizeInMap",
    std::bind(&GlobalRobotLocalization::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalRobotLocalization::handleCancel, this, std::placeholders::_1),
    std::bind(&GlobalRobotLocalization::handleAccepted, this, std::placeholders::_1));
}

void GlobalRobotLocalization::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  const auto options = readRuntimeOptions();
  MapBuildOptions map_options;
  map_options.occupied_threshold = options.occupied_threshold;
  map_options.unknown_is_occupied = options.unknown_is_occupied;
  map_options.map_padding_xy = options.search.map_padding_xy;

  MapModel updated_map;
  if (!updated_map.update(*msg, map_options)) {
    RCLCPP_WARN(get_logger(), "Ignoring invalid occupancy map");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = std::move(updated_map);
  }
  RCLCPP_INFO(
    get_logger(), "Updated map model: %ux%u at %.3f m/cell",
    msg->info.width, msg->info.height, msg->info.resolution);
}

void GlobalRobotLocalization::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  scans_.push_back(*msg);
  while (scans_.size() > max_scan_buffer_size_) {
    scans_.pop_front();
  }
}

rclcpp_action::GoalResponse GlobalRobotLocalization::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const LocalizeInMap::Goal>)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!map_.ready()) {
    RCLCPP_WARN(get_logger(), "Rejecting localization goal because no map has been received");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalRobotLocalization::handleCancel(
  const std::shared_ptr<GoalHandleLocalizeInMap>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalRobotLocalization::handleAccepted(
  const std::shared_ptr<GoalHandleLocalizeInMap> goal_handle)
{
  std::thread{std::bind(&GlobalRobotLocalization::executeLocalization, this, goal_handle)}.detach();
}

void GlobalRobotLocalization::executeLocalization(
  const std::shared_ptr<GoalHandleLocalizeInMap> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<LocalizeInMap::Result>();

  MapModel map_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_snapshot = map_;
  }

  if (!map_snapshot.ready()) {
    result->success = false;
    result->message = "no map has been received";
    goal_handle->abort(result);
    return;
  }

  const auto options = readRuntimeOptions();
  auto scans = collectScans(goal_handle, options, goal);
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "localization canceled";
    goal_handle->canceled(result);
    return;
  }

  if (scans.empty()) {
    result->success = false;
    result->message = "timed out waiting for lidar scans";
    goal_handle->abort(result);
    return;
  }

  auto feedback = std::make_shared<LocalizeInMap::Feedback>();
  feedback->state = "searching";
  feedback->collected_scans = static_cast<std::uint32_t>(scans.size());
  goal_handle->publish_feedback(feedback);

  feedback = std::make_shared<LocalizeInMap::Feedback>();
  feedback->state = "transforming_scans";
  feedback->collected_scans = static_cast<std::uint32_t>(scans.size());
  goal_handle->publish_feedback(feedback);

  std::string transform_error;
  const auto endpoints = scansToBaseEndpoints(scans, options, transform_error);
  if (!transform_error.empty()) {
    result->success = false;
    result->message = transform_error;
    goal_handle->abort(result);
    return;
  }

  feedback = std::make_shared<LocalizeInMap::Feedback>();
  feedback->state = "searching";
  feedback->collected_scans = static_cast<std::uint32_t>(scans.size());
  goal_handle->publish_feedback(feedback);

  LocalizationSearch search(options.search);
  const auto search_result = search.run(map_snapshot, endpoints);
  result->success = search_result.success;
  result->message = search_result.message;
  result->cost = search_result.best.cost;
  result->scan_count = static_cast<std::uint32_t>(scans.size());

  if (!search_result.success) {
    goal_handle->abort(result);
    return;
  }

  result->pose = makePoseMessage(
    map_snapshot.frame_id(),
    now(),
    search_result.best.pose.x,
    search_result.best.pose.y,
    search_result.best.pose.yaw,
    search_result.covariance);

  const bool publish_initialpose = options.publish_initialpose && !goal->suppress_initialpose;
  const bool publish_markers = options.publish_markers && !goal->suppress_markers;
  if (publish_initialpose) {
    initialpose_pub_->publish(result->pose);
  }
  if (publish_markers) {
    publishCandidateMarkers(map_snapshot.frame_id(), result->pose.header.stamp, search_result.candidates);
  }

  goal_handle->succeed(result);
}

GlobalRobotLocalization::RuntimeOptions GlobalRobotLocalization::readRuntimeOptions() const
{
  RuntimeOptions options;
  options.occupied_threshold = static_cast<int>(get_parameter("occupied_threshold").as_int());
  options.unknown_is_occupied = get_parameter("unknown_is_occupied").as_bool();
  options.base_frame = get_parameter("base_frame").as_string();
  options.num_scans = static_cast<int>(get_parameter("num_scans").as_int());
  options.scan_collection_timeout_sec = get_parameter("scan_collection_timeout_sec").as_double();
  options.transform_timeout_sec = get_parameter("transform_timeout_sec").as_double();
  options.search.coarse_xy_step = get_parameter("coarse_xy_step").as_double();
  options.search.coarse_yaw_step = get_parameter("coarse_yaw_step_deg").as_double() * kPi / 180.0;
  options.search.top_k = static_cast<int>(get_parameter("top_k").as_int());
  options.search.candidate_min_xy_separation =
    get_parameter("candidate_min_xy_separation").as_double();
  options.search.candidate_min_yaw_separation =
    get_parameter("candidate_min_yaw_separation_deg").as_double() * kPi / 180.0;
  options.search.refine_levels = static_cast<int>(get_parameter("refine_levels").as_int());
  options.search.refine_xy_step = get_parameter("refine_xy_step").as_double();
  options.search.refine_yaw_step = get_parameter("refine_yaw_step_deg").as_double() * kPi / 180.0;
  options.search.scan_stride = static_cast<int>(get_parameter("scan_stride").as_int());
  options.search.max_range = get_parameter("max_range").as_double();
  options.search.off_map_distance = get_parameter("off_map_distance").as_double();
  options.search.sigma_r = get_parameter("sigma_r").as_double();
  options.search.sigma_theta = get_parameter("sigma_theta").as_double();
  options.search.alpha_m = get_parameter("alpha_m").as_double();
  options.search.map_padding_xy = get_parameter("map_padding_xy").as_double();
  options.search.free_space_weight = get_parameter("free_space_weight").as_double();
  options.search.free_space_sample_step = get_parameter("free_space_sample_step").as_double();
  options.search.min_endpoint_count = static_cast<int>(get_parameter("min_endpoint_count").as_int());
  options.search.covariance_step_xy = get_parameter("covariance_step_xy").as_double();
  options.search.covariance_step_yaw =
    get_parameter("covariance_step_yaw_deg").as_double() * kPi / 180.0;
  options.search.covariance_regularization = get_parameter("covariance_regularization").as_double();
  options.search.covariance_scale = get_parameter("covariance_scale").as_double();
  options.publish_initialpose = get_parameter("publish_initialpose").as_bool();
  options.publish_markers = get_parameter("publish_markers").as_bool();
  options.num_scans = std::max(options.num_scans, 1);
  if (options.base_frame.empty()) {
    options.base_frame = "base_link";
  }
  return options;
}

std::vector<sensor_msgs::msg::LaserScan> GlobalRobotLocalization::collectScans(
  const std::shared_ptr<GoalHandleLocalizeInMap> & goal_handle,
  const RuntimeOptions & options,
  const std::shared_ptr<const LocalizeInMap::Goal> & goal)
{
  const int requested_scans = goal->num_scans > 0 ?
    static_cast<int>(goal->num_scans) : options.num_scans;
  const double timeout_sec = isZeroDuration(goal->collection_timeout) ?
    options.scan_collection_timeout_sec : secondsFromDuration(goal->collection_timeout);
  const auto deadline = now() + rclcpp::Duration::from_seconds(std::max(0.0, timeout_sec));

  std::vector<sensor_msgs::msg::LaserScan> collected;
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok() && now() <= deadline && !goal_handle->is_canceling()) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (scans_.size() >= static_cast<std::size_t>(requested_scans)) {
        collected.assign(scans_.end() - requested_scans, scans_.end());
        break;
      }
      collected.assign(scans_.begin(), scans_.end());
    }

    auto feedback = std::make_shared<LocalizeInMap::Feedback>();
    feedback->state = "collecting_scans";
    feedback->collected_scans = static_cast<std::uint32_t>(collected.size());
    goal_handle->publish_feedback(feedback);
    rate.sleep();
  }

  if (collected.size() > static_cast<std::size_t>(requested_scans)) {
    collected.erase(collected.begin(), collected.end() - requested_scans);
  }
  return collected.size() >= static_cast<std::size_t>(requested_scans) ? collected :
         std::vector<sensor_msgs::msg::LaserScan>{};
}

LaserScanPoints GlobalRobotLocalization::scansToBaseEndpoints(
  const std::vector<sensor_msgs::msg::LaserScan> & scans,
  const RuntimeOptions & options,
  std::string & error_message)
{
  LaserScanPoints laserScanPoints;
  const int scan_stride = std::max(options.search.scan_stride, 1);
  const auto timeout = rclcpp::Duration::from_seconds(std::max(0.0, options.transform_timeout_sec));

  double variance_r = options.search.sigma_r * options.search.sigma_r;
  double variance_theta = options.search.sigma_theta * options.search.sigma_theta;

  for (const auto & scan : scans) {
    if (scan.header.frame_id.empty()) {
      error_message = "scan header frame_id is empty";
      return {};
    }

    const bool identity_transform = scan.header.frame_id == options.base_frame;
    geometry_msgs::msg::TransformStamped transform;
    tf2::Matrix3x3 rotation;
    double tx = 0.0;
    double ty = 0.0;

    if (!identity_transform) {
      try {
        transform = tf_buffer_.lookupTransform(
          options.base_frame,
          scan.header.frame_id,
          scan.header.stamp,
          timeout);
      } catch (const tf2::TransformException & ex) {
        error_message = "failed to transform scan from '" + scan.header.frame_id + "' to '" +
          options.base_frame + "': " + ex.what();
        return {};
      }

      const auto & q_msg = transform.transform.rotation;
      tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      q.normalize();
      rotation = tf2::Matrix3x3(q);
      tx = transform.transform.translation.x;
      ty = transform.transform.translation.y;
    }

    const double usable_max_range = options.search.max_range > 0.0 ?
      std::min(options.search.max_range, static_cast<double>(scan.range_max)) :
      static_cast<double>(scan.range_max);

    for (std::size_t i = 0; i < scan.ranges.size(); i += static_cast<std::size_t>(scan_stride)) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > usable_max_range) {
        continue;
      }

      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(i) * static_cast<double>(scan.angle_increment);
      const double laser_x = static_cast<double>(range) * std::cos(angle);
      const double laser_y = static_cast<double>(range) * std::sin(angle);

      laserScanPoints.ranges.emplace_back(range);
      if (identity_transform) {
        laserScanPoints.endpoints.emplace_back(laser_x, laser_y);
      } else {
        laserScanPoints.endpoints.emplace_back(
          rotation[0][0] * laser_x + rotation[0][1] * laser_y + tx,
          rotation[1][0] * laser_x + rotation[1][1] * laser_y + ty);
      }

      double lidar_variance = variance_r + range * range * variance_theta;

      laserScanPoints.variances.emplace_back(lidar_variance);
    }
  }

  return laserScanPoints;
}

geometry_msgs::msg::PoseWithCovarianceStamped GlobalRobotLocalization::makePoseMessage(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double x,
  double y,
  double yaw,
  const Eigen::Matrix3d & covariance) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = covariance(0, 0);
  msg.pose.covariance[1] = covariance(0, 1);
  msg.pose.covariance[5] = covariance(0, 2);
  msg.pose.covariance[6] = covariance(1, 0);
  msg.pose.covariance[7] = covariance(1, 1);
  msg.pose.covariance[11] = covariance(1, 2);
  msg.pose.covariance[30] = covariance(2, 0);
  msg.pose.covariance[31] = covariance(2, 1);
  msg.pose.covariance[35] = covariance(2, 2);
  msg.pose.covariance[14] = 1.0e6;
  msg.pose.covariance[21] = 1.0e6;
  msg.pose.covariance[28] = 1.0e6;
  return msg;
}

void GlobalRobotLocalization::publishCandidateMarkers(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::vector<Candidate> & candidates)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "global_localization_candidates";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  const std::size_t count = std::min<std::size_t>(candidates.size(), 50);
  for (std::size_t i = 0; i < count; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "global_localization_candidates";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = candidates[i].pose.x;
    marker.pose.position.y = candidates[i].pose.y;
    marker.pose.position.z = 0.05;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, candidates[i].pose.yaw);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.4;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.a = 0.85F;
    marker.color.r = i == 0 ? 0.1F : 1.0F;
    marker.color.g = i == 0 ? 1.0F : 0.6F;
    marker.color.b = i == 0 ? 0.1F : 0.0F;
    markers.markers.push_back(marker);
  }
  marker_pub_->publish(markers);
}

}  // namespace global_robot_localization
