cmake_minimum_required(VERSION 3.8)
project(tnb_mns_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

set(msg_files
  "msg/TnbMnsStatus.msg"
)

set(srv_files
  "srv/Enable.srv"
  "srv/RunRegular.srv"
  "srv/RunResonant.srv"
  "srv/Stop.srv"
  "srv/TNBMNSStateTransition.srv")

add_executable(tnb_mns_driver_node src/tnb_mns_driver_node.cpp)
ament_target_dependencies(tnb_mns_driver_node rclcpp std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
    DEPENDENCIES std_msgs
)

#allows to use services from within the same package
# rosidl_target_interfaces(tnb_mns_driver_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(tnb_mns_driver_node "${cpp_typesupport_target}")

install(TARGETS
  tnb_mns_driver_node
  DESTINATION lib/${PROJECT_NAME})

# ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)



ament_package()

#


# catkin_install_python(PROGRAMS 
#     scripts/calibration_tnbmns.py
#     scripts/current_control.py
#     scripts/calibration_centerpoint_tnbmns.py
#     scripts/driver_test.py
#     scripts/field_control.py
#     scripts/field_explorer.py
#     scripts/saturation_tests.py
#     scripts/resonance_tester.py
#     scripts/tnb_mns_calculator_node.py
#     scripts/tnb_emns_python_control.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB python_scripts scripts/*.py)