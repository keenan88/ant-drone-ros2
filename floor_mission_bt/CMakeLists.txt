cmake_minimum_required(VERSION 3.8)
project(floor_mission_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ant_fleet_interfaces REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(behaviortree_ros2)


# Install directories
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB AUTONOMY_SOURCES src/bt_cpp_nodes/*.cpp)

set(TARGET_DEPENDS
    ament_index_cpp 
    rclcpp 
    rclcpp_action
    behaviortree_cpp 
    std_msgs
    rmf_task_msgs
    nlohmann_json
    ant_fleet_interfaces
    rmf_fleet_msgs
    linkattacher_msgs
    example_interfaces
    behaviortree_ros2
)
include_directories(include)

add_executable(floor_mission_bt src/floor_mission_bt.cpp ${AUTONOMY_SOURCES})

install(TARGETS
floor_mission_bt
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_target_dependencies(floor_mission_bt ${TARGET_DEPENDS})
target_link_libraries(
  floor_mission_bt 
)

ament_package()
