cmake_minimum_required(VERSION 3.5)
project(antdrone_gz)

find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs)
find_package(domain_bridge)
find_package(linkattacher_msgs REQUIRED)
find_package(rosgraph_msgs)
find_package(nav_msgs)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs)
find_package(geometry_msgs)
find_package(std_msgs)
find_package(tf2_msgs)

set(TARGET_DEPENDS
    gazebo_msgs
    domain_bridge
    linkattacher_msgs
    rosgraph_msgs
    nav_msgs
    nav2_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    tf2_msgs
)

add_executable(gz_ros_domain_bridge src/gz_ros_domain_bridge.cpp)
add_executable(gz_frame_name_fixer src/gz_frame_name_fixer.cpp)

install(TARGETS
    gz_ros_domain_bridge
    gz_frame_name_fixer
    DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(gz_ros_domain_bridge ${TARGET_DEPENDS})
ament_target_dependencies(gz_frame_name_fixer ${TARGET_DEPENDS})


ament_package()
