cmake_minimum_required(VERSION 3.8)
project(antdrone_bt)

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
find_package(ant_queen_interfaces REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(behaviortree_ros2)
find_package(tf2_ros)
find_package(geometry_msgs)
find_package(domain_bridge)
find_package(nav2_msgs)
find_package(antdrone_interfaces)

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
    ant_queen_interfaces
    rmf_fleet_msgs
    linkattacher_msgs
    example_interfaces
    behaviortree_ros2
    tf2_ros
    geometry_msgs
    domain_bridge
    nav2_msgs
    antdrone_interfaces
)
include_directories(include)

add_executable(antdrone_bt src/antdrone_bt.cpp ${AUTONOMY_SOURCES})

add_executable(queen_domain_bridge src/queen_domain_bridge.cpp ${AUTONOMY_SOURCES})



install(TARGETS
    antdrone_bt
    queen_domain_bridge
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(antdrone_bt ${TARGET_DEPENDS})

ament_target_dependencies(queen_domain_bridge ${TARGET_DEPENDS})


target_link_libraries(
  antdrone_bt 
)

ament_package()
