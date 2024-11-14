cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(SNO)

set(SUITESPARSE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd/SuiteSparse-7.8.3/build/install/include")
set(SUITESPARSE_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd/SuiteSparse-7.8.3/build/install/lib")
set(CUDA_INCLUDE_DIR "/usr/local/cuda/include")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#find_package(Eigen3 REQUIRED)

include_directories(./src/)
include_directories(./3rd/eigen-3.4.0/)

include_directories(${SUITESPARSE_INCLUDE_DIR})
include_directories(${SUITESPARSE_INCLUDE_DIR}/suitesparse)
include_directories(${CUDA_INCLUDE_DIR})

set(SUITESPARSE_LIBS
    ${SUITESPARSE_LIB_DIR}/libcholmod.so
    ${SUITESPARSE_LIB_DIR}/libamd.so
    ${SUITESPARSE_LIB_DIR}/libcamd.so
    ${SUITESPARSE_LIB_DIR}/libcolamd.so
    ${SUITESPARSE_LIB_DIR}/libccolamd.so
    ${SUITESPARSE_LIB_DIR}/libsuitesparseconfig.so
)

file(GLOB_RECURSE SRC ./src/*.cpp)
file(GLOB_RECURSE ANN_SRC ./src/ANN/*.cpp)
file(GLOB_RECURSE HLBFGS_SRC ./src/HLBFGS/*.cpp)
list(FILTER SRC EXCLUDE REGEX "main.cpp")

add_executable(${PROJECT_NAME} ./src/main.cpp ${SRC} ${ANN_SRC} ${HLBFGS_SRC})

#target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE ${SUITESPARSE_LIBS})
