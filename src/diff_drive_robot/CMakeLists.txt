cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/yolo_tracker.py
  scripts/sort.py
  scripts/semantic_navigator.py
  scripts/arrow_teleop.py
  scripts/voice_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
