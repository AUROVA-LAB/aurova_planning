cmake_minimum_required(VERSION 3.5)
project(dynamic_obstacles_prediction)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(detection_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(dynamic_obstacles_prediction_node  src/dynamic_obstacles_prediction.cpp)
# target_link_libraries(dynamic_obstacles_prediction_node ${PCL_LIBRARIES})
ament_target_dependencies(dynamic_obstacles_prediction_node rclcpp  Eigen3 detection_msgs  nav_msgs PCL pcl_conversions geometry_msgs ackermann_msgs)

install(TARGETS
  dynamic_obstacles_prediction_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp Eigen3)
ament_package()






