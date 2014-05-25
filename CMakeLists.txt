project(HaoRegistration)
cmake_minimum_required(VERSION 2.8)

find_package(PCL REQUIRED common)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories($ENV{LEMON_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(srcs    src/main.cpp
            src/point_cloud.cpp
            src/graph_map.cpp
            src/parameter_map.cpp
            src/solver.cpp)
            
set(incs    include/point_cloud.h
            include/graph_map.h
            include/parameter_map.h
            include/solver.h)

add_executable(HaoRegistration ${srcs} ${incs})

target_link_libraries(HaoRegistration ${PCL_COMMON_LIBRARY})

install(TARGETS HaoRegistration RUNTIME DESTINATION bin)
