cmake_minimum_required(VERSION 3.5)
project(xmobot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(xmobot_launch)

install(DIRECTORY urdf worlds rviz launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(mount_geometry test/test_mount_geometry.py)
endif()

ament_package()
