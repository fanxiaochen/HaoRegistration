project(HaoRegistration)
cmake_minimum_required(VERSION 2.8)

find_package(OpenSceneGraph REQUIRED osgViewer osgGA)
find_package(PCL REQUIRED common kdtree search features)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenNI REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories($ENV{LEMON_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(srcs    src/main.cpp
            src/point_cloud.cpp
            src/graph_map.cpp
            src/parameter_map.cpp
            src/solver.cpp
            src/visualizer.cpp
            src/event_handler.cpp
            src/scene_viewer.cpp)
            
set(incs    include/point_cloud.h
            include/graph_map.h
            include/parameter_map.h
            include/solver.h
            include/visualizer.h
            include/event_handler.h
            include/scene_viewer.h)

add_executable(HaoRegistration ${srcs} ${incs})

target_link_libraries(HaoRegistration ${OPENSCENEGRAPH_LIBRARIES} ${PCL_COMMON_LIBRARY} ${PCL_KDTREE_LIBRARY} ${PCL_FEATURES_LIBRARY} ${PCL_SEARCH_LIBRARY} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${OPENNI_LIBRARIES})

install(TARGETS HaoRegistration RUNTIME DESTINATION bin)
