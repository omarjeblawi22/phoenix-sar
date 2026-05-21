cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY config description launch worlds maps phoenixIMPPF
  DESTINATION share/${PROJECT_NAME}
)

# SAR autonomous exploration node
install(PROGRAMS
  scripts/phoenix_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phoenix_explorer
)

# Camera target detection node (Pi Camera + TFLite)
install(PROGRAMS
  scripts/camera_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_detector
)

# Web dashboard node (serves http://<pi>:8080)
install(PROGRAMS
  scripts/phoenix_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phoenix_dashboard
)

# Web launcher node (serves http://<pi>:8081)
install(PROGRAMS
  scripts/phoenix_launcher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phoenix_launcher
)

# Circle patrol node
install(PROGRAMS
  scripts/phoenix_circle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phoenix_circle
)

# Forward navigation node (3m ahead, obstacle avoiding)
install(PROGRAMS
  scripts/phoenix_forward.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phoenix_forward
)

# IMPPF dataset logger (reads XIAO ESP32-S3 via /dev/ttyACM0 + SLAM topics)
install(PROGRAMS
  phoenixIMPPF/phoenix_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME phoenix_logger
)

ament_package()
