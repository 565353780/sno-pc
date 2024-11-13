cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "-std=c++17 -O0")
project(SNO)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(D_GLIBCXX_USE_CXX11_ABI 0)

include_directories(./src/)
include_directories(./src/ANN/)
include_directories(./src/HLBFGS/)

file(GLOB_RECURSE SRC ./src/*.cpp)
file(GLOB_RECURSE ANN_SRC ./src/ANN/*.cpp)
file(GLOB_RECURSE HLBFGS_SRC ./src/HLBFGS/*.cpp)
list(FILTER SRC EXCLUDE REGEX "main.cpp")

add_executable(${PROJECT_NAME} ./src/main.cpp ${SRC} ${ANN_SRC} ${HLBFGS_SRC})
