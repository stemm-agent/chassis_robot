cmake_minimum_required(VERSION 3.5)
project(wheeltec_nav2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Keep production builds optimized while still retaining useful symbols.  Do
# not overwrite an explicit colcon/CMake selection (for example Release).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(amcl_tf_guard src/amcl_tf_guard.cpp)
ament_target_dependencies(amcl_tf_guard
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  std_msgs
  std_srvs
)

add_executable(amcl_noise_scheduler src/amcl_noise_scheduler.cpp)
ament_target_dependencies(amcl_noise_scheduler
  rclcpp
  nav_msgs
  sensor_msgs
)

add_executable(nav2_guard_interlock src/nav2_guard_interlock.cpp)
ament_target_dependencies(nav2_guard_interlock
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_nav2_tf_guard_contract
    test/test_nav2_tf_guard_contract.py
  )
  ament_add_pytest_test(
    test_amcl_tf_guard_odom_contract
    test/test_amcl_tf_guard_odom_contract.py
  )
  ament_add_pytest_test(
    test_nav2_guard_interlock_contract
    test/test_nav2_guard_interlock_contract.py
  )
  ament_add_pytest_test(
    test_nav2_launch_singleton_contract
    test/test_nav2_launch_singleton_contract.py
  )
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY behavior_trees launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS amcl_tf_guard
  amcl_noise_scheduler
  nav2_guard_interlock
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
