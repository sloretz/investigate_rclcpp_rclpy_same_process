cmake_minimum_required(VERSION 3.10)
project(rccl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(pybind11_vendor)
find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

pybind11_add_module(rccl SHARED
  use_ros.cpp
)
target_link_libraries(rccl PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

# Sets PYTHON_INSTALL_DIR
_ament_cmake_python_get_python_install_dir()

install(
  TARGETS rccl
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

ament_package()
