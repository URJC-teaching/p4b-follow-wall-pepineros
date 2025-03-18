cmake_minimum_required(VERSION 3.8)
project(p4b-follow-wall-pepineros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
)

include_directories(include)

add_library(FollowWall SHARED
  src/FollowWall/FollowWall.cpp
)
ament_target_dependencies(FollowWall ${dependencies})

add_executable(follow_wall_main src/FollowWallMain.cpp)
ament_target_dependencies(follow_wall_main ${dependencies})
target_link_libraries(follow_wall_main FollowWall)

install(TARGETS
  ${PROJECT_NAME}
  follow_wall_main
  FollowWall
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
