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

add_executable(pickup_node src/pickup_node.cpp)
target_link_libraries(pickup_node ${LIBUSB_LIBRARIES} ${LIBDAQ_LIBRARIES})
ament_target_dependencies(pickup_node rclcpp std_msgs)

install(TARGETS
  pickup_node
  DESTINATION lib/${PROJECT_NAME})

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
