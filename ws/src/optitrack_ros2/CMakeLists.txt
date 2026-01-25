cmake_minimum_required(VERSION 3.8)
project(optitrack_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)



link_directories(src/lib)
include_directories(src/include)
# Create executable
add_executable(optitrack_node src/optitrack_node.cpp)

ament_target_dependencies(optitrack_node
  rclcpp
  geometry_msgs
)

# Link NatNet library
target_link_libraries(optitrack_node
  NatNet
)

# Install executable
install(TARGETS
  optitrack_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()