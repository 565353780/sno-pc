cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(SNO)

set(SUITESPARSE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd/SuiteSparse-7.8.3/build/install/include")
set(SUITESPARSE_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd/SuiteSparse-7.8.3/build/install/lib")
set(CUDA_INCLUDE_DIR "/usr/local/cuda/include")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-stack-protector")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-stack-protector")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#find_package(MKL REQUIRED)
find_package(OpenMP REQUIRED)

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

#target_link_libraries(${PROJECT_NAME} PRIVATE MKL::MKL)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE ${SUITESPARSE_LIBS})
