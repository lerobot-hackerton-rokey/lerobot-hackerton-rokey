cmake_minimum_required(VERSION 3.8)
project(rokey_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ────── 의존 패키지 찾기 ──────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ────── Executable 1: fk_publisher (단발 FK 테스트) ──────
add_executable(fk_publisher
  src/fk_publisher.cpp
)
ament_target_dependencies(fk_publisher
  rclcpp
  urdf
  moveit_core
  moveit_ros_planning_interface
  ament_index_cpp
)

# ────── Executable 2: fk_list_publisher (JointState 구독 → Pose 퍼블리시) ──────
add_executable(fk_list_publisher
  src/fk_list_publisher.cpp
)
ament_target_dependencies(fk_list_publisher
  rclcpp
  urdf
  moveit_core
  moveit_ros_planning_interface
  ament_index_cpp
  sensor_msgs
  geometry_msgs
)

# ────── 설치 설정 ──────
install(TARGETS
  fk_publisher
  fk_list_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
