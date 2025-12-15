cmake_minimum_required(VERSION 3.8)
project(my_quadruped_cpp_pkg)

# Derleyici ayarları
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ Standardı
add_compile_options(-std=c++17)

# 1. Paketleri Bulma (Find Packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)        # Bunu ekledim (Target'larda kullanmissin)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
# Not: rclpy sildim çünkü bu C++ paketi, python kütüphanesine ihtiyacı yok.

# 2. Executable: Gait Node
add_executable(simple_gait_node src/simple_gait_node.cpp)
ament_target_dependencies(simple_gait_node 
  rclcpp 
  geometry_msgs 
  std_msgs
)

# 3. Executable: IK Node
add_executable(simple_ik_node src/simple_ik_node.cpp)
ament_target_dependencies(simple_ik_node 
  rclcpp 
  std_msgs 
  trajectory_msgs
  geometry_msgs # Genelde IK işlemlerinde geometry_msgs de lazım olur, garanti olsun diye ekledim.
)

# 4. Yükleme (Install)
install(TARGETS 
  simple_gait_node 
  simple_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
