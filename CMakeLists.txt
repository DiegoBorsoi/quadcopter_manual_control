cmake_minimum_required(VERSION 3.8)
project(quadcopter_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(include)

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "std_msgs" "std_srvs")

add_executable(quad_control src/quad_control.cpp)
ament_target_dependencies(quad_control ${dependencies})
target_compile_features(quad_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(quad_multi_control-linux-SDL src/quad_multi_control-linux-SDL.cpp)
ament_target_dependencies(quad_multi_control-linux-SDL ${dependencies})
target_link_libraries(quad_multi_control-linux-SDL ${SDL2_LIBRARIES})
target_compile_features(quad_multi_control-linux-SDL PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS quad_control
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS quad_multi_control-linux-SDL
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
