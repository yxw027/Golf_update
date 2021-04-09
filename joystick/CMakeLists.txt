cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(joystick)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Use a different C++ compiler.")
endif()

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF() 

include_directories( /usr/local/include/ssmtype ../OMcntl/src ../utility/src ../viewer/src ../config/src ../imu_rt/src ../gnss-f9p/src ../localizer/src ../navigate/src)

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

add_executable(manualOperation ./src/manualOperation.cpp ../OMcntl/src/ModbusRTU.cpp ./src/operateJoystick.cpp ../utility/src/utility.cpp)
target_link_libraries(manualOperation ssm rppjs)

add_executable(handle-viewer ./src/handle-viewer.cpp ./src/GraphDrawerHandle.cpp ../utility/src/utility.cpp ../utility/src/log2txt.cpp)
target_link_libraries(handle-viewer m ssm)

add_executable(accel-viewer ./src/accel-viewer.cpp ./src/GraphDrawerAccel.cpp ../utility/src/utility.cpp ../utility/src/log2txt.cpp)
target_link_libraries(accel-viewer m ssm)

add_executable(lever-viewer ./src/lever-viewer.cpp ./src/GraphDrawerLever.cpp ../utility/src/utility.cpp ../utility/src/log2txt.cpp)
target_link_libraries(lever-viewer m ssm)

add_executable(recordOperation ./src/recordOperation.cpp ../OMcntl/src/ModbusRTU.cpp ./src/operateJoystick.cpp ../utility/src/utility.cpp)
target_link_libraries(recordOperation ssm)


