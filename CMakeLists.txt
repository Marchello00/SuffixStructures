cmake_minimum_required(VERSION 3.13)
project(RefrainSolver)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -O1 -fno-omit-frame-pointer -g")

add_executable(RefrainSingleFile single_file/single_file_for_contest.cpp)
add_executable(Refrain main.cpp)