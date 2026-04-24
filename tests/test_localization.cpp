#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "global_robot_localization/localization_search.hpp"
#include "global_robot_localization/map_model.hpp"

namespace global_robot_localization
{
namespace
{

struct GeneratedMap
{
  nav_msgs::msg::OccupancyGrid msg;
};

struct SimulatedLidar
{
  LaserScanPoints endpoints_base;
};

double yawError(double lhs, double rhs)
{
  return std::abs(normalizeYaw(lhs - rhs));
}

GeneratedMap makeEmptyMap(std::uint32_t width, std::uint32_t height, double resolution)
{
  GeneratedMap map;
  map.msg.header.frame_id = "map";
  map.msg.info.resolution = static_cast<float>(resolution);
  map.msg.info.width = width;
  map.msg.info.height = height;
  map.msg.info.origin.orientation.w = 1.0;
  map.msg.data.assign(static_cast<std::size_t>(width) * height, 0);
  return map;
}

void setCell(GeneratedMap & map, int x, int y, std::int8_t value)
{
  if (x < 0 || y < 0 ||
    x >= static_cast<int>(map.msg.info.width) ||
    y >= static_cast<int>(map.msg.info.height))
  {
    return;
  }
  map.msg.data[static_cast<std::size_t>(y) * map.msg.info.width + x] = value;
}

void setOccupiedRectCells(GeneratedMap & map, int min_x, int min_y, int max_x, int max_y)
{
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      setCell(map, x, y, 100);
    }
  }
}

void setOccupiedRectWorld(
  GeneratedMap & map,
  double min_x,
  double min_y,
  double max_x,
  double max_y)
{
  const double resolution = map.msg.info.resolution;
  setOccupiedRectCells(
    map,
    static_cast<int>(std::floor(min_x / resolution)),
    static_cast<int>(std::floor(min_y / resolution)),
    static_cast<int>(std::floor(max_x / resolution)),
    static_cast<int>(std::floor(max_y / resolution)));
}

void addMapBorder(GeneratedMap & map)
{
  const int width = static_cast<int>(map.msg.info.width);
  const int height = static_cast<int>(map.msg.info.height);
  setOccupiedRectCells(map, 0, 0, width - 1, 0);
  setOccupiedRectCells(map, 0, height - 1, width - 1, height - 1);
  setOccupiedRectCells(map, 0, 0, 0, height - 1);
  setOccupiedRectCells(map, width - 1, 0, width - 1, height - 1);
}

GeneratedMap makeAsymmetricMap()
{
  auto map = makeEmptyMap(90, 70, 0.1);
  addMapBorder(map);

  setOccupiedRectWorld(map, 4.8, 0.9, 5.0, 4.6);
  setOccupiedRectWorld(map, 1.1, 4.0, 1.9, 4.8);
  setOccupiedRectWorld(map, 3.1, 1.0, 3.4, 1.9);
  setOccupiedRectWorld(map, 6.3, 4.4, 7.2, 5.3);
  setOccupiedRectWorld(map, 7.6, 1.0, 7.9, 2.6);
  setOccupiedRectWorld(map, 2.2, 5.6, 2.8, 5.9);
  setOccupiedRectWorld(map, 5.8, 2.1, 6.1, 2.4);
  setOccupiedRectWorld(map, 0.9, 1.2, 1.2, 2.0);
  setOccupiedRectWorld(map, 3.9, 5.0, 4.5, 5.2);
  return map;
}

void addSymmetricRoom(GeneratedMap & map, double origin_x, double origin_y)
{
  setOccupiedRectWorld(map, origin_x, origin_y, origin_x + 4.0, origin_y + 0.1);
  setOccupiedRectWorld(map, origin_x, origin_y + 4.0, origin_x + 4.0, origin_y + 4.1);
  setOccupiedRectWorld(map, origin_x, origin_y, origin_x + 0.1, origin_y + 4.1);
  setOccupiedRectWorld(map, origin_x + 4.0, origin_y, origin_x + 4.1, origin_y + 4.1);
  setOccupiedRectWorld(map, origin_x + 1.0, origin_y + 2.8, origin_x + 1.6, origin_y + 3.4);
  setOccupiedRectWorld(map, origin_x + 2.7, origin_y + 0.9, origin_x + 2.9, origin_y + 2.1);
}

GeneratedMap makePathologicallySymmetricMap()
{
  auto map = makeEmptyMap(130, 60, 0.1);
  addSymmetricRoom(map, 1.0, 1.0);
  addSymmetricRoom(map, 8.0, 1.0);
  return map;
}

bool isOccupied(const nav_msgs::msg::OccupancyGrid & map, double wx, double wy)
{
  const auto x = static_cast<int>(std::floor(wx / map.info.resolution));
  const auto y = static_cast<int>(std::floor(wy / map.info.resolution));
  if (x < 0 || y < 0 ||
    x >= static_cast<int>(map.info.width) ||
    y >= static_cast<int>(map.info.height))
  {
    return true;
  }
  return map.data[static_cast<std::size_t>(y) * map.info.width + x] >= 50;
}

SimulatedLidar simulateLidar(
  const nav_msgs::msg::OccupancyGrid & map,
  const Pose2D & robot_pose,
  SearchOptions& options,
  int beam_count = 181,
  double field_of_view = 2.0 * kPi,
  double max_range = 6.0)
{

  double variance_r = options.sigma_r * options.sigma_r;
  double variance_theta = options.sigma_theta * options.sigma_theta;

  SimulatedLidar scan;
  const double min_angle = -0.5 * field_of_view;
  const double angle_step = field_of_view / static_cast<double>(beam_count - 1);
  const double ray_step = 0.5 * map.info.resolution;

  for (int i = 0; i < beam_count; ++i) {
    const double beam_angle_base = min_angle + static_cast<double>(i) * angle_step;
    const double beam_angle_map = robot_pose.yaw + beam_angle_base;
    const double cos_a = std::cos(beam_angle_map);
    const double sin_a = std::sin(beam_angle_map);

    for (double range = ray_step; range <= max_range; range += ray_step) {
      const double wx = robot_pose.x + range * cos_a;
      const double wy = robot_pose.y + range * sin_a;
      if (isOccupied(map, wx, wy)) {
        scan.endpoints_base.endpoints.emplace_back(
          range * std::cos(beam_angle_base),
          range * std::sin(beam_angle_base));
        scan.endpoints_base.ranges.emplace_back(range);
        double lidar_variance = variance_r + range * range * variance_theta;
        scan.endpoints_base.variances.emplace_back(lidar_variance);
        break;
      }
    }
  }

  return scan;
}

std::vector<Eigen::Vector2d> transformEndpointsToMap(
  const std::vector<Eigen::Vector2d> & endpoints_base,
  const Pose2D & pose)
{
  std::vector<Eigen::Vector2d> endpoints_map;
  endpoints_map.reserve(endpoints_base.size());
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (const auto & endpoint : endpoints_base) {
    endpoints_map.emplace_back(
      pose.x + c * endpoint.x() - s * endpoint.y(),
      pose.y + s * endpoint.x() + c * endpoint.y());
  }
  return endpoints_map;
}

MapModel buildMapModel(const nav_msgs::msg::OccupancyGrid & map)
{
  MapModel model;
  MapBuildOptions options;
  options.occupied_threshold = 50;
  options.unknown_is_occupied = true;
  EXPECT_TRUE(model.update(map, options));
  return model;
}

SearchOptions makeFunctionalTestSearchOptions()
{
  SearchOptions options;
  options.coarse_xy_step = 0.1;
  options.coarse_yaw_step = 5.0 * kPi / 180.0;
  options.top_k = 40;
  options.candidate_min_xy_separation = 0.45;
  options.candidate_min_yaw_separation = 8.0 * kPi / 180.0;
  options.refine_levels = 4;
  options.refine_xy_step = 0.15;
  options.refine_yaw_step = 4.0 * kPi / 180.0;
  options.off_map_distance = 3.0;
  options.free_space_weight = 0.4;
  options.free_space_sample_step = 0.2;
  options.min_endpoint_count = 80;
  options.covariance_regularization = 1.0e-2;
  options.sigma_r = 0.03;
  options.sigma_theta = 0.005;
  options.alpha_m = 1.0;
  return options;
}

std::filesystem::path testArtifactDirectory()
{
  const char * test_tmpdir = std::getenv("TEST_TMPDIR");
  if (test_tmpdir != nullptr && std::string(test_tmpdir).size() > 0) {
    std::filesystem::create_directories(test_tmpdir);
    return test_tmpdir;
  }

  auto path = std::filesystem::temp_directory_path() / "global_robot_localization_tests";
  std::filesystem::create_directories(path);
  return path;
}

void writeGnuplotArtifacts(
  const std::string & name,
  const std::string & title,
  const nav_msgs::msg::OccupancyGrid & map,
  const MapModel & model,
  LocalizationSearch & search,
  const Pose2D & truth,
  const std::vector<Eigen::Vector2d> & endpoints_base,
  const SearchResult & result)
{
  const auto dir = testArtifactDirectory();
  const auto cost_path = dir / (name + "_yaw_min_cost.dat");
  const auto occupied_path = dir / (name + "_occupied.dat");
  const auto poses_path_truth = dir / (name + "_poses_truth.dat");
  const auto poses_path_est = dir / (name + "_poses_est.dat");
  const auto endpoints_path = dir / (name + "_lidar_endpoints_map.dat");
  const auto script_path = dir / (name + ".gp");
  const auto png_path = dir / (name + ".png");

  {
    search.dumpCoarseScore(cost_path);
  }

  {
    std::ofstream occupied(occupied_path);
    for (std::uint32_t y = 0; y < map.info.height; ++y) {
      for (std::uint32_t x = 0; x < map.info.width; ++x) {
        if (map.data[static_cast<std::size_t>(y) * map.info.width + x] < 50) {
          continue;
        }
        occupied << (static_cast<double>(x) + 0.5) * map.info.resolution << ' '
                 << (static_cast<double>(y) + 0.5) * map.info.resolution << '\n';
      }
    }
  }

  {
    std::ofstream poses_truth(poses_path_truth);
    poses_truth << truth.x << ' ' << truth.y << ' '
           << 0.5 * std::cos(truth.yaw) << ' ' << 0.5 * std::sin(truth.yaw)
           << " truth\n";
    std::ofstream poses_est(poses_path_est);
    if (result.success) {
      poses_est << result.best.pose.x << ' ' << result.best.pose.y << ' '
            << 0.35 * std::cos(result.best.pose.yaw) << ' '
            << 0.35 * std::sin(result.best.pose.yaw) << " estimated\n";
    }
  }

  {
    std::ofstream endpoints(endpoints_path);
    const auto endpoints_map = transformEndpointsToMap(endpoints_base, truth);
    for (const auto & endpoint : endpoints_map) {
      endpoints << endpoint.x() << ' ' << endpoint.y() << '\n';
    }
  }

  {
    //double step = search.xy_step_cells * search.resolution;
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1000,700\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title '" << title << "'\n";
    script << "set key outside\n";
    script << "set palette defined (0 '#f7fcf0', 0.5 '#addd8e', 1 '#2b8cbe')\n";
    script << "set cbrange [20:1000]\n";
    script << "set cblabel 'min J over yaw'\n";
    script << "set style fill transparent solid 0.4\n"; // 40% opaque
 
    // Start the plot command
    script << "plot '" << cost_path.string() << "' with image title 'cost', \\\n";
    script << "     '" << occupied_path.string() << "' using 1:2 with points pt 5 ps 0.35 lc rgb '#303030' title 'occupied', \\\n";
    script << "     '" << poses_path_truth.string() << "' every ::0::0 using 1:2:3:4 with vectors head filled lw 3 lc rgb '#1f77b4' title 'truth', \\\n";
    script << "     '" << poses_path_est.string() << "' every ::0::0 using 1:2:3:4 with vectors head filled lw 3 lc rgb '#2ca02c' title 'estimated', \\\n";
    script << "     '" << endpoints_path.string() << "' using 1:2 with points pt 7 ps 0.45 lc rgb '#d62728' title 'lidar endpoints'\n";
  }

  const std::string command = "gnuplot '" + script_path.string() + "' >/dev/null 2>&1";
  static_cast<void>(std::system(command.c_str()));
}

bool containsPoseNear(
  const std::vector<Candidate> & candidates,
  const Pose2D & pose,
  double xy_tolerance,
  double yaw_tolerance)
{
  return std::any_of(candidates.begin(), candidates.end(), [&](const Candidate & candidate) {
    const double dx = candidate.pose.x - pose.x;
    const double dy = candidate.pose.y - pose.y;
    return std::hypot(dx, dy) <= xy_tolerance &&
           yawError(candidate.pose.yaw, pose.yaw) <= yaw_tolerance;
  });
}

TEST(LocalizationFunctionalTest, LocalizesAsymmetricMap)
{
  // this test places graphical output in:
  // /tmp/global_robot_localization_tests/asymmetric_localization.png
  // it is useful to watch this file in your ide (or otherwise)
  auto options = makeFunctionalTestSearchOptions();

  const auto generated = makeAsymmetricMap();
  const auto model = buildMapModel(generated.msg);
  const Pose2D truth{2.45, 2.25, 0.65};
  const auto scan = simulateLidar(generated.msg, truth, options, 91, 2.0 * kPi, 5.5);
  ASSERT_GE(scan.endpoints_base.endpoints.size(), 70U);

  LocalizationSearch search(options);
  const auto result = search.run(model, scan.endpoints_base);
  const double truth_cost = search.scorePose(model, scan.endpoints_base, truth);
  writeGnuplotArtifacts(
    "asymmetric_localization",
    "asymmetric localization",
    generated.msg, model, search, truth,
    scan.endpoints_base.endpoints, result);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(std::hypot(result.best.pose.x - truth.x, result.best.pose.y - truth.y), 0.18);
  std::cout << "estimated pose: x=" << result.best.pose.x << " y=" << result.best.pose.y
    << " yaw=" << result.best.pose.yaw << " cost=" << result.best.cost
    << " truth_cost=" << truth_cost << std::endl;
  EXPECT_LT(yawError(result.best.pose.yaw, truth.yaw), 5.0 * kPi / 180.0);
  std::cout << "covariance: " << result.covariance  << std::endl;

  EXPECT_TRUE(result.covariance.allFinite());
  EXPECT_GT(result.covariance(0, 0), 0.0);
  EXPECT_GT(result.covariance(1, 1), 0.0);
  EXPECT_GT(result.covariance(2, 2), 0.0);
}

TEST(LocalizationFunctionalTest, ExposesMultipleGlobalMinimaInSymmetricMap)
{
  // this test places graphical output in:
  // /tmp/global_robot_localization_tests/symmetric_localization.png
  // it is useful to watch this file in your ide (or otherwise)
  auto options = makeFunctionalTestSearchOptions();
  const auto generated = makePathologicallySymmetricMap();
  const auto model = buildMapModel(generated.msg);
  const Pose2D truth{2.8, 2.6, 0.35};
  const Pose2D equivalent{9.8, 2.6, 0.35};
  const auto scan = simulateLidar(generated.msg, truth, options, 91, 2.0 * kPi, 4.0);
  ASSERT_GE(scan.endpoints_base.endpoints.size(), 70U);

  options.top_k = 24;
  LocalizationSearch search(options);
  const auto result = search.run(model, scan.endpoints_base);
  const double truth_cost = search.scorePose(model, scan.endpoints_base, truth);
  writeGnuplotArtifacts(
    "symmetric_localization",
    "symmetric localization",
    generated.msg, model, search, truth,
    scan.endpoints_base.endpoints, result);

  std::cout << "estimated pose: x=" << result.best.pose.x << " y=" << result.best.pose.y
    << " yaw=" << result.best.pose.yaw << " cost=" << result.best.cost
    << " truth_cost=" << truth_cost << std::endl;    
  std::cout << "covariance: " << result.covariance  << std::endl;  

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(containsPoseNear(result.candidates, truth, 0.25, 6.0 * kPi / 180.0));
  EXPECT_TRUE(containsPoseNear(result.candidates, equivalent, 0.25, 6.0 * kPi / 180.0));
  ASSERT_GE(result.candidates.size(), 2U);
  EXPECT_LT(std::abs(result.candidates.front().cost - result.candidates[1].cost), 0.02);
}

}  // namespace
}  // namespace global_robot_localization
