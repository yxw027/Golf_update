cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(viewer)

if(UNIX)
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
endif()

include_directories(../config/src ../navigate/src ../gnss-f9p/src ../imu_rt/src ../OMcntl/src ../localizer/src ../utility/src ../viewer/src)

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

add_executable(navi-viewer ./src/navi-viewer.cpp ./src/GraphDrawerTrajectory.cpp ../navigate/src/wp-mgr.cpp ../utility/src/log2txt.cpp ../utility/src/utility.cpp)
target_link_libraries(navi-viewer m ssm)

add_executable(control-log2txt ./src/control-log2txt.cpp ../utility/src/log2txt.cpp ../utility/src/utility.cpp)
target_link_libraries(control-log2txt m ssm)

add_executable(WP-viewer ./src/WP-viewer.cpp ./src/GraphDrawerWP.cpp ../navigate/src/wp-mgr.cpp ../utility/src/utility.cpp)
target_link_libraries(WP-viewer m ssm)

add_executable(vel-viewer ./src/vel-viewer.cpp ./src/GraphDrawerVelocity.cpp ../utility/src/utility.cpp  ../utility/src/log2txt.cpp)
target_link_libraries(vel-viewer m ssm)
