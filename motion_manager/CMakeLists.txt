cmake_minimum_required(VERSION 3.8)
project(motion_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nao_robocup_interfaces REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  nao_robocup_interfaces)

#build1

add_library(${PROJECT_NAME}_node SHARED
  src/motion_manager_node.cpp
)

add_library(pub_test_node SHARED
  src/pub_test.cpp
)

target_include_directories(${PROJECT_NAME}_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(pub_test_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)  

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "motion_manager::MotionManagerNode"
  EXECUTABLE ${PROJECT_NAME})

rclcpp_components_register_node(pub_test_node
  PLUGIN "motion_manager::PubTest"
  EXECUTABLE pub_test)


ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(pub_test_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(
  TARGETS pub_test_node
  EXPORT export_pub_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Package
ament_package()


#build2

#add_executable(motion_manager_node src/motion_manager_node.cpp)
#ament_target_dependencies(motion_manager_node rclcpp std_msgs nao_robocup_interfaces)

#add_executable(motion_manager_pub src/pub_test.cpp)
#ament_target_dependencies(motion_manager_pub rclcpp nao_robocup_interfaces)

#install(TARGETS
#  motion_manager_node
#  DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

#ament_package()
