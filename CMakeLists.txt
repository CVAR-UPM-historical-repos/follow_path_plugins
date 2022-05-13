cmake_minimum_required(VERSION 3.5)
project(follow_path_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  follow_path_plugin_base
  rclcpp
  rclcpp_action
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(follow_path_plugins src/follow_path_plugin_traj.cpp)
target_include_directories(follow_path_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  follow_path_plugins
  ${PROJECT_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(follow_path_plugin_base plugins.xml)

install(
  TARGETS follow_path_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  follow_path_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
