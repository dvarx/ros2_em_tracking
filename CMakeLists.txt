cmake_minimum_required(VERSION 3.8)
project(mdriver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

set(msg_files
  "msg/Status.msg"
)

set(srv_files
  "srv/Enable.srv"
  "srv/RunRegular.srv"
  "srv/RunResonant.srv"
  "srv/Stop.srv"
  "srv/StateTransition.srv")

add_executable(mdriver_node src/mdriver_node.cpp)
add_executable(mdriver_node_test src/mdriver_node_test.cpp)
ament_target_dependencies(mdriver_node rclcpp std_msgs)
ament_target_dependencies(mdriver_node_test rclcpp std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
    DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mdriver_node "${cpp_typesupport_target}")
target_link_libraries(mdriver_node_test "${cpp_typesupport_target}")

install(TARGETS
  mdriver_node
  mdriver_node_test
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()

file(GLOB python_scripts scripts/*.py)