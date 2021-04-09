cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(navigate)

include_directories(./src /usr/local/include/ssmtype ../config/src ../imu_rt/src ../detectObstacle/src ../localizer/src ../utility/src ../OMcntl/src )

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set the build type
set(CMAKE_BUILD_TYPE Release)
# confirmation messages
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

add_executable(navigate ./src/navigate.cpp ./src/system-mgr.cpp ./src/wp-mgr.cpp ../OMcntl/src/ModbusRTU.cpp ../utility/src/utility.cpp)
target_link_libraries(navigate m ssm)

