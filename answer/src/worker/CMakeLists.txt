cmake_minimum_required(VERSION 3.8)
project(worker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
set(AMENT_DEPENDENCIES
  rclcpp
  cv_bridge
  sensor_msgs
  OpenCV
)

foreach (AMENT_DEPENDENCY ${AMENT_DEPENDENCIES})
    find_package(${AMENT_DEPENDENCY} REQUIRED)
endforeach ()


add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
        include
        serialpro
        msg_serializer)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
