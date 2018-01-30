cmake_minimum_required(VERSION 3.9)
project(INE)

set(CMAKE_CXX_STANDARD 14)

add_executable(INE main.cpp src/ClassType.cpp src/BeautyCode.cpp src/INE.cpp)

include_directories(include)