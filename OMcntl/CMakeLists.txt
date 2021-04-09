cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(OMcntl)

include_directories(./src ../config/src ../utility/src ../imu_rt/src ../navigate/src ../localizer/src)

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

add_executable(OMcntl ./src/OMcntl.cpp ./src/ModbusRTU.cpp ../utility/src/utility.cpp)
target_link_libraries(OMcntl m ssm)

