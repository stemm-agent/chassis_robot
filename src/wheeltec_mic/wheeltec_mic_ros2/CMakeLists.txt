cmake_minimum_required(VERSION 3.8)
project(wheeltec_mic_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(WHEELTEC_MIC_LIB_SUBDIR arm64)
else()
  set(WHEELTEC_MIC_LIB_SUBDIR x64)
endif()

set(WHEELTEC_MIC_VENDOR_LIB_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${WHEELTEC_MIC_LIB_SUBDIR}
)

# Copy vendor libraries into the build tree and normalize their runtime lookup.
# The supplied x64 offline recorder library contains a Jetson-era absolute
# RUNPATH; using $ORIGIN keeps the installed pair self-contained on the NUC.
find_program(PATCHELF_EXECUTABLE patchelf REQUIRED)
set(WHEELTEC_MIC_VENDOR_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/vendor_lib
)
file(MAKE_DIRECTORY ${WHEELTEC_MIC_VENDOR_BUILD_DIR})
foreach(WHEELTEC_MIC_VENDOR_LIBRARY liboffline_record_lib.so libmsc.so)
  configure_file(
    ${WHEELTEC_MIC_VENDOR_LIB_DIR}/${WHEELTEC_MIC_VENDOR_LIBRARY}
    ${WHEELTEC_MIC_VENDOR_BUILD_DIR}/${WHEELTEC_MIC_VENDOR_LIBRARY}
    COPYONLY
  )
endforeach()
execute_process(
  COMMAND ${PATCHELF_EXECUTABLE} --set-rpath "$ORIGIN"
    ${WHEELTEC_MIC_VENDOR_BUILD_DIR}/liboffline_record_lib.so
  RESULT_VARIABLE WHEELTEC_MIC_PATCHELF_RESULT
  ERROR_VARIABLE WHEELTEC_MIC_PATCHELF_ERROR
)
if(NOT WHEELTEC_MIC_PATCHELF_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Failed to normalize liboffline_record_lib.so RUNPATH: "
    "${WHEELTEC_MIC_PATCHELF_ERROR}"
  )
endif()

link_directories(
  ${WHEELTEC_MIC_VENDOR_BUILD_DIR}
)
#ament_index_register_resource("ros1_bridge_foreign_mapping")
install(DIRECTORY launch config feedback_voice
  DESTINATION share/${PROJECT_NAME})

# find dependencies
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(wheeltec_mic_msg REQUIRED)
find_package(turn_on_wheeltec_robot REQUIRED)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(call_recognition src/call_recognition.cpp)
ament_target_dependencies(call_recognition rclcpp std_msgs wheeltec_mic_msg)

add_executable(command_recognition src/command_recognition.cpp)
ament_target_dependencies(command_recognition rclcpp rclcpp_action std_msgs wheeltec_mic_msg geometry_msgs nav2_msgs action_msgs)
target_link_libraries(command_recognition rt)

add_executable(node_feedback src/node_feedback.cpp)
ament_target_dependencies(node_feedback rclcpp std_msgs geometry_msgs)

add_executable(wheeltec_m2n src/wheeltec_m2n.cpp src/ros2_service_m2n.cpp)
ament_target_dependencies(wheeltec_m2n rclcpp std_msgs serial wheeltec_mic_msg)
target_link_libraries(wheeltec_m2n jsoncpp)

add_executable(wheeltec_mic src/wheeltec_mic.cpp src/ros2_service.cpp)
ament_target_dependencies(wheeltec_mic rclcpp std_msgs serial wheeltec_mic_msg)
target_link_libraries(wheeltec_mic jsoncpp)

add_executable(wheeltec_m07 src/wheeltec_m07.cpp)
ament_target_dependencies(wheeltec_m07 rclcpp std_msgs serial)

add_executable(voice_control src/voice_control.cpp)
ament_target_dependencies(voice_control ament_index_cpp rclcpp std_msgs wheeltec_mic_msg)
target_link_libraries(voice_control
  -Wl,--no-as-needed
  offline_record_lib
  msc
  -Wl,--as-needed
  rt dl pthread stdc++ asound samplerate
)
set_target_properties(voice_control PROPERTIES
  BUILD_RPATH "${WHEELTEC_MIC_VENDOR_BUILD_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

add_executable(motion_control src/motion_control.cpp)
ament_target_dependencies(motion_control rclcpp rclcpp_action std_msgs wheeltec_mic_msg geometry_msgs turn_on_wheeltec_robot nav_msgs tf2 tf2_ros tf2_geometry_msgs nav2_msgs)

install(PROGRAMS
  scripts/voice_recharge_bridge
  scripts/voice_cartographer_bridge
  scripts/nav2_start_feedback_monitor
  scripts/voice_summon_coordinator
  scripts/voice_summon_coordinator_bearing_nav
  scripts/voice_summon_bearing_nav_progress_watchdog
  scripts/voice_summon_bearing_nav_yolo_preempt
  scripts/voice_summon_shadow_logger
  scripts/voice_summon_bearing_search_shadow
  scripts/voice_summon_pre_nav_shadow
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  scripts/cartographer_voice_policy.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  ${WHEELTEC_MIC_VENDOR_BUILD_DIR}/liboffline_record_lib.so
  ${WHEELTEC_MIC_VENDOR_BUILD_DIR}/libmsc.so
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  call_recognition
  command_recognition
  node_feedback
  wheeltec_mic
  wheeltec_m2n
  wheeltec_m07
  voice_control
  motion_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}    
)

ament_package()
