project(HaoRegistration)
cmake_minimum_required(VERSION 2.8)

include_directories($ENV{EIGEN3_INCLUDE_DIR})
include_directories($ENV{LEMON_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(srcs    src/main.cpp
            src/point_cloud.cpp)
            
set(incs    include/point_cloud.h)

add_executable(HaoRegistration ${srcs} ${incs})

install(TARGETS HaoRegistration RUNTIME DESTINATION bin)
