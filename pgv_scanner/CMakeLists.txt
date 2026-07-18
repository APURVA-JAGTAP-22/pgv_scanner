cmake_minimum_required(VERSION 3.8)
project(pgv_scanner)

# 1. Standard C++ Settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



# --- PGV Driver ---
add_executable(pgv_driver src/pgv_driver.cpp)
ament_target_dependencies(pgv_driver
  rclcpp
  std_msgs
  geometry_msgs
)



# 5. Install Rules
install(TARGETS
  pgv_driver
 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 6. Finalize Package
ament_export_dependencies(rclcpp std_msgs geometry_msgs)
ament_package()
