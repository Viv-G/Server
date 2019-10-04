cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_VERSION VERSION_LESS 3.8)
  # CMake did not have cxx_std_14 compile feature prior to 3.8
  # We use cxx_attribute_deprecated as a proxy because this feature is a part of c++14 standard
  set(PCL_CXX_COMPILE_FEATURES cxx_attribute_deprecated)
else()
  set(PCL_CXX_COMPILE_FEATURES cxx_std_14)
endif()

project(pois)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pois pois.cpp)
target_link_libraries (pois ${PCL_LIBRARIES})