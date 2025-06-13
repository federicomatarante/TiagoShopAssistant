cmake_minimum_required(VERSION 3.8)
project(tiago)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Get all msg files with relative paths
file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg") # <-- ADD THIS LINE

# Get all srv files with relative paths
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

# Generate interfaces (services and messages)
if(srv_files OR msg_files) # <-- MODIFIED THIS LINE
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}   # <-- ADD THIS LINE
    ${srv_files}
    DEPENDENCIES geometry_msgs std_msgs
  )
endif()

# Install Python package
# ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY
  tiago/
  DESTINATION ${PYTHON_INSTALL_DIR}/tiago
  FILES_MATCHING PATTERN "*.py"
)
# Install binary files (images) without any processing
install(
  DIRECTORY res/
  DESTINATION share/${PROJECT_NAME}/res
)
# Install Python executables

install(PROGRAMS
  ${PROJECT_NAME}/map_provider.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_provider
)
install(PROGRAMS
  ${PROJECT_NAME}/map_tester.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_tester
)
# Install the maps directory to the share folder
install(
  DIRECTORY tiago/maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(
  DIRECTORY tiago/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(PROGRAMS
  ${PROJECT_NAME}/database_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME database_interface
)
install(PROGRAMS
  ${PROJECT_NAME}/test_database_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_database_node
)
install(PROGRAMS
  ${PROJECT_NAME}/path_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME path_planner
)


# Install launch files if they exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# Install config files if they exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Testing
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

  # Python tests - only if tests directory exists
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(pytest tests/
      TIMEOUT 300
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
endif()

ament_package()