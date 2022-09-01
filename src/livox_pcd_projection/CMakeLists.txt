cmake_minimum_required(VERSION 3.8)
project(livox_pcd_projection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set (CMAKE_CXX_STANDARD 11)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(livox_interfaces REQUIRED)
# find_package(livox_pcd_projection REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(vision_msgs REQUIRED) # for bbox msg
find_package(dist_msg REQUIRED) # for custom distance message


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(projection src/projectCloud.cpp)
target_include_directories(projection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(projection rclcpp PCL OpenCV cv_bridge pcl_ros image_transport std_msgs livox_interfaces vision_msgs dist_msg)


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

install(TARGETS
  projection
  DESTINATION lib/${PROJECT_NAME})

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install (DIRECTORY config
#   DESTINATION share/${PROJECT_NAME})