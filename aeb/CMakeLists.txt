cmake_minimum_required(VERSION 3.8)
project(aeb)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories("/usr/include/eigen3")

include_directories(include)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node
  rclcpp
  sensor_msgs
  std_msgs
)    

add_executable(safety_node_test test/safety_node.cpp) 
ament_target_dependencies(safety_node_test
  rclcpp
  std_msgs
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS safety_node safety_node_test 
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION lib/${PROJECT_NAME}
) 

ament_package()
