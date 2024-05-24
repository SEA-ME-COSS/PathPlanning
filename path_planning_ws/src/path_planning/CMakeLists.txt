cmake_minimum_required(VERSION 3.5)
project(path_planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

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

include_directories(
  include/${PROJECT_NAME}/a_star
  include/${PROJECT_NAME}/bfs
)

# bfs node
add_executable(bfs
  src/bfs/path_planning.cpp
  src/bfs/main.cpp
  src/bfs/utils/map.cpp
  src/bfs/utils/loader.cpp
  src/bfs/planner/planner.cpp
  src/bfs/planner/bfs.cpp
  src/bfs/decision_making/decision_making.cpp
)

ament_target_dependencies(bfs
  rclcpp std_msgs vision_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs example_interfaces
)

# a_star node
add_executable(a_star
  src/a_star/path_planning.cpp
  src/a_star/main.cpp
  src/a_star/utils/map.cpp
  src/a_star/utils/loader.cpp
  src/a_star/planner/planner.cpp
  src/a_star/planner/a_star.cpp
  src/a_star/decision_making/decision_making.cpp
)

ament_target_dependencies(a_star
  rclcpp std_msgs vision_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs example_interfaces
)

install(TARGETS
  bfs
  a_star
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()