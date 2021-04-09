cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gnss-f9p)

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

include_directories( ./src ../config/src ../utility/src ../viewer/src ../imu_rt/src ../localizer/src ../navigate/src ../OMcntl/src)

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

add_executable(gnss-f9p-handler ./src/gnss-f9p-handler.cpp ./src/rtknavi-f9p.cpp ../utility/src/utility.cpp)
target_link_libraries(gnss-f9p-handler m ssm)

add_executable(gnss-f9p-viewer ./src/gnss-f9p-viewer.cpp ./src/GraphDrawerGNSS-f9p.cpp  ./src/rtknavi-f9p.cpp ../utility/src/utility.cpp ../utility/src/log2txt.cpp)
target_link_libraries(gnss-f9p-viewer m ssm)

