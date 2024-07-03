cmake_minimum_required(VERSION 3.8)
project(mocap4r2_robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    mocap4r2_msgs
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
    nav_msgs
)

include_directories(
  include
)

add_library(localization_component SHARED src/mocap4r2_robot_localization/localization_component.cpp)
ament_target_dependencies(localization_component ${dependencies})
rclcpp_components_register_nodes(localization_component "mocap4r2_robot_localization::LocalizationNode")

add_executable(localization_program src/localization_program.cpp)
ament_target_dependencies(localization_component ${dependencies})
target_link_libraries(localization_program localization_component)


install(TARGETS
  localization_component
  localization_program
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
