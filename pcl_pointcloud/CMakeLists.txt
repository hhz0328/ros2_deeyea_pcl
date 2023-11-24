cmake_minimum_required(VERSION 3.5)
project(pcl_pointcloud)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters features kdtree surface)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(points_demo src/points_demo.cpp)
target_include_directories(points_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  points_demo
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "rcl_interfaces"
  "rcutils"
  "std_msgs"
  "tf2_geometry_msgs"
)

add_executable(points_once src/points_once.cpp)
target_include_directories(points_once PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  points_once
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "rcl_interfaces"
  "rcutils"
  "std_msgs"
  "tf2_geometry_msgs"
)

add_executable(surface src/surface.cpp)
target_include_directories(surface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  surface
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "rcl_interfaces"
  "rcutils"
  "std_msgs"
  "tf2_geometry_msgs"
)

add_executable(visual src/visual.cpp)
target_include_directories(visual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  visual
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "rcl_interfaces"
  "rcutils"
  "std_msgs"
  "tf2_geometry_msgs"
)

add_executable(features src/features.cpp)
target_include_directories(features PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  features
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "rcl_interfaces"
  "rcutils"
  "std_msgs"
  "tf2_geometry_msgs"
)

target_link_libraries(points_demo
  ${PCL_LIBRARIES}  # pcl_conversions requires this
)

target_link_libraries(points_once
  ${PCL_LIBRARIES}
)

target_link_libraries(surface
  ${PCL_LIBRARIES}
)

target_link_libraries(visual
  ${PCL_LIBRARIES}
)

target_link_libraries(features
  ${PCL_LIBRARIES}
)

install(TARGETS points_demo
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS points_once
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS surface
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS visual
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS features
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
