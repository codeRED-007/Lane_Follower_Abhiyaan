cmake_minimum_required(VERSION 3.8)
project(path_finder  LANGUAGES C CXX CUDA)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(custom_msgs)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(CUDA REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(IPM_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ipm.cu
)

# ament_python_install_package(${PROJECT_NAME})
add_executable(ipm
  ${IPM_SRC}
)

set_property(TARGET ipm PROPERTY CUDA_ARCHITECTURES "87")


ament_target_dependencies(ipm
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  CUDA
  OpenCV
)

install(TARGETS ipm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/path_publisher.py

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY 
    src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(
  ${PCL_INCLUDE_DIRS}
)