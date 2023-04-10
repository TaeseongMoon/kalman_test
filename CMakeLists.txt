cmake_minimum_required(VERSION 3.12)
project(kalman_filter)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
	pybind11
	GIT_REPOSITORY https://github.com/pybind/pybind11.git
	GIT_TAG        v2.6.2
	GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pybind11)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(kalman_filter SHARED kalman_filter.cpp bindings.cpp)
target_link_libraries(kalman_filter PRIVATE pybind11::module Eigen3::Eigen)