cmake_minimum_required(VERSION 3.8)
project(my_quadruped_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)              
find_package(tf2_ros REQUIRED)          
find_package(tf2_geometry_msgs REQUIRED) 
find_package(nav_msgs REQUIRED)

# --- simple_gait_node ---
add_executable(simple_gait_node src/simple_gait_node.cpp)
ament_target_dependencies(simple_gait_node 
  rclcpp 
  geometry_msgs 
  std_msgs
  sensor_msgs 
  champ_msgs
  visualization_msgs
  tf2                  
  tf2_ros              
  tf2_geometry_msgs    
)

# --- simple_ik_node ---
add_executable(simple_ik_node src/simple_ik_node.cpp)
ament_target_dependencies(simple_ik_node 
  rclcpp 
  std_msgs 
  trajectory_msgs
  geometry_msgs
)

# --- teleop ---
add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop 
  rclcpp 
  geometry_msgs
)

install(TARGETS 
  simple_gait_node 
  simple_ik_node
  teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
