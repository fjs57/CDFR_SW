cmake_minimum_required(VERSION 3.8)
project(hw_if_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# include directories
include_directories(
  include/hw_if_pkg
  CDFR_COM_LIB/nanoPB/common 
  CDFR_COM_LIB/nanoPB/messages
  CDFR_COM_LIB/TinyFrame
)


ament_package()
