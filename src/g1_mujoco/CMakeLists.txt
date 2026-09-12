cmake_minimum_required(VERSION 3.16)
project(g1_mujoco)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch mjcf scenes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
