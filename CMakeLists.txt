cmake_minimum_required(VERSION 3.8)
project(ros2_humanoid_virtual_twin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)#?????

#zadeklaruj liste wiadomosci ktora chcesz wygenerowac:
set(srv_files
  "srv/Legmove.srv"
)

#Wygeneruj wiadomosci:
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)

#Tworzenie pliko wykonywalnych
add_executable(move_leg config/move_leg.cpp)
ament_target_dependencies(move_leg rclcpp sensor_msgs)#????

add_executable(client config/client.cpp)
ament_target_dependencies(client rclcpp)

install(TARGETS
  move_leg
  client
  DESTINATION lib/${PROJECT_NAME}
)

#Poniewaz masz w 1 paczce interfejsy to:
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(move_leg "${cpp_typesupport_target}")
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(client "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds STLe
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
