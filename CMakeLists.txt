cmake_minimum_required(VERSION 3.8)
project(global_robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LocalizeInMap.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

set(CORE_LIBRARY_TARGET ${PROJECT_NAME}_core)

add_library(${CORE_LIBRARY_TARGET}
  src/global_robot_localization.cpp
  src/localization_search.cpp
  src/map_model.cpp
  src/sim.cpp
  src/sim_node.cpp
)
target_compile_features(${CORE_LIBRARY_TARGET} PUBLIC cxx_std_17)
target_include_directories(${CORE_LIBRARY_TARGET} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${CORE_LIBRARY_TARGET}
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(${CORE_LIBRARY_TARGET}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}"
)

add_executable(global_robot_localization_node
  src/global_robot_localization_node.cpp
)
target_compile_features(global_robot_localization_node PUBLIC cxx_std_17)
target_link_libraries(global_robot_localization_node
  ${CORE_LIBRARY_TARGET}
)
ament_target_dependencies(global_robot_localization_node
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

install(
  TARGETS ${CORE_LIBRARY_TARGET}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS global_robot_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${CORE_LIBRARY_TARGET})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rosidl_default_runtime
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_localization
    tests/test_localization.cpp
    tests/test_node.cpp
  )
  if(TARGET test_localization)
    target_compile_features(test_localization PUBLIC cxx_std_17)
    target_link_libraries(test_localization
      ${CORE_LIBRARY_TARGET}
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
