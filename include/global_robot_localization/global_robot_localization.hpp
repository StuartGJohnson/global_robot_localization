#ifndef GLOBAL_ROBOT_LOCALIZATION__GLOBAL_ROBOT_LOCALIZATION_HPP_
#define GLOBAL_ROBOT_LOCALIZATION__GLOBAL_ROBOT_LOCALIZATION_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "global_robot_localization/action/localize_in_map.hpp"
#include "global_robot_localization/localization_search.hpp"
#include "global_robot_localization/map_model.hpp"

namespace global_robot_localization
{

class GlobalRobotLocalization : public rclcpp::Node
{
public:
  using LocalizeInMap = global_robot_localization::action::LocalizeInMap;
  using GoalHandleLocalizeInMap = rclcpp_action::ServerGoalHandle<LocalizeInMap>;

  explicit GlobalRobotLocalization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct RuntimeOptions;

  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const LocalizeInMap::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleLocalizeInMap> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleLocalizeInMap> goal_handle);
  void executeLocalization(const std::shared_ptr<GoalHandleLocalizeInMap> goal_handle);

  RuntimeOptions readRuntimeOptions() const;
  std::vector<sensor_msgs::msg::LaserScan> collectScans(
    const std::shared_ptr<GoalHandleLocalizeInMap> & goal_handle,
    const RuntimeOptions & options,
    const std::shared_ptr<const LocalizeInMap::Goal> & goal);
  LaserScanPoints scansToBaseEndpoints(
    const std::vector<sensor_msgs::msg::LaserScan> & scans,
    const RuntimeOptions & options,
    std::string & error_message);
  geometry_msgs::msg::PoseWithCovarianceStamped makePoseMessage(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    double x,
    double y,
    double yaw,
    const Eigen::Matrix3d & covariance) const;
  void publishCandidateMarkers(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std::vector<Candidate> & candidates);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Server<LocalizeInMap>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  mutable std::mutex data_mutex_;
  MapModel map_;
  std::deque<sensor_msgs::msg::LaserScan> scans_;
  std::size_t max_scan_buffer_size_{100};
};

}  // namespace global_robot_localization

#endif  // GLOBAL_ROBOT_LOCALIZATION__GLOBAL_ROBOT_LOCALIZATION_HPP_
