cmake_minimum_required(VERSION 3.8)
project(mass_spring_damper_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(customs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(state_space_node src/state_space_node.cpp)
ament_target_dependencies(state_space_node rclcpp customs example_interfaces)

install(TARGETS
  state_space_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
