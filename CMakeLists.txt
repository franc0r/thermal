cmake_minimum_required(VERSION 3.8)
project(francor_thermal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(seek_node
  src/seek_node.cpp
  src/SeekThermalCameraNode.cpp
)
target_include_directories(seek_node
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(seek_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(seek_node
  rclcpp
  cv_bridge
  image_transport
)
target_link_libraries(seek_node
  seek
  ${OpenCV_LIBS}
)

# Install node(s)
install(TARGETS seek_node
  DESTINATION lib/${PROJECT_NAME})

# Install all launch files.
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install all config files.
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

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
