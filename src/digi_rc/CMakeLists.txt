cmake_minimum_required(VERSION 3.8)
project(digi_rc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  DEPENDENCIES std_msgs
)

# Install Python executables
install(PROGRAMS
  src/commander.py
  src/single_drone.py
  src/multi_drone.py
  mock_mission/mockmissionplanner.py
  mock_mission/multi_wayn.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()