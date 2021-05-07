cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(urg)

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

find_package(PCL 1.1 REQUIRED)
include_directories(../config/src ../utility/src ../viewer/src ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_executable(urg-handler ./src/urg-handler.cpp ./src/lidar.cpp ../utility/src/utility.cpp)
target_link_libraries(urg-handler m ssm scip2awd)

add_executable(urg-viewer ./src/urg-viewer.cpp ./src/GraphDrawerURG.cpp ../utility/src/utility.cpp)
target_link_libraries(urg-viewer m ssm)

add_executable(urg-3Dviewer ./src/urg-3Dviewer.cpp ./src/GraphDrawerPCD.cpp ../utility/src/log2txt.cpp ../utility/src/utility.cpp)
target_link_libraries(urg-3Dviewer m ssm ${PCL_LIBRARIES})
