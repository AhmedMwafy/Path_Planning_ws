cmake_minimum_required(VERSION 3.8)
project(graph_search_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

add_executable(a_star_graph_search src/a_star_planner.cpp)
add_executable(dijkstra_graph_search src/dijkstra_planner.cpp)

ament_target_dependencies(dijkstra_graph_search ${dependencies})
ament_target_dependencies(a_star_graph_search ${dependencies})


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  dijkstra_graph_search
  a_star_graph_search
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()

