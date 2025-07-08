cmake_minimum_required(VERSION 3.8)
project(em_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBDAQ REQUIRED libuldaq)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

include_directories(include ${LIBUSB_DIRECTORIES} ${LIBDAQ_INCLUDE_DIRECTORIES})

add_executable(pickup_node src/pickup_node.cpp src/errstrs.c)
target_link_libraries(pickup_node ${LIBUSB_LIBRARIES} ${LIBDAQ_LIBRARIES})
ament_target_dependencies(pickup_node rclcpp std_msgs)

#install executables
install(TARGETS
  pickup_node
  DESTINATION lib/${PROJECT_NAME})

#install launch files
install(DIRECTORY ${em_tracking_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME})

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  em_tracking_py/measurement_node.py
  em_tracking_py/measurement_node_cal.py
  em_tracking_py/signal_test.py
  em_tracking_py/em_calibration_stepper_node.py
  em_tracking_py/localization_node.py
  emns_utils/magnetic_system.py
  emns_utils/magnetic_tracking.py
  stages_utils/three_axis_stage.py
  stages_utils/utilities.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
