project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.h src/FusionEKF.h src/kalman_filter.h src/ekf_test.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(test/googletest/googletest/include)
include_directories(test/googletest/googlemock/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
add_subdirectory(src/googletest)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(ExtendedKF ${sources})

target_link_libraries(ExtendedKF z ssl uv uWS)
target_link_libraries(ExtendedKF gtest gtest_main)
target_link_libraries(ExtendedKF gmock gmock_main)




