cmake_minimum_required(VERSION 3.12)

project(ddnmpc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()
message(STATUS "Generating with build type: ${CMAKE_BUILD_TYPE}")

find_package(PkgConfig REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(benchmark REQUIRED)
pkg_check_modules(CPPAD  REQUIRED  cppad)
pkg_check_modules(IPOPT  REQUIRED  ipopt)

include_directories(include)

link_libraries(
    ${IPOPT_LIBRARIES}
    ${CPPAD_LIBRARIES}
)

set(SOURCE_FILES
    src/mpc.cpp
    src/model.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

add_executable(bm_nmpc benchmark/bm_nmpc.cpp)
target_link_libraries(bm_nmpc 
    PRIVATE ${PROJECT_NAME} benchmark
)
