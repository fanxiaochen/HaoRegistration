project(HaoRegistration)

include_directories($ENV{EIGEN3_INCLUDE_DIR})

add_executable(HaoRegistration main.cpp)

install(TARGETS HaoRegistration RUNTIME DESTINATION bin)
