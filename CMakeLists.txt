cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Golf)

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

include_directories(./OMcntl/src ./config/src ./createWP/src ./detectObstacle/src 
./gnss-f9p/src ./imu_rt/src ./joystick/src ./localizer/src ./navigate/src 
./urg/src ./utility/src ./viewer/src /usr/local/include/ssmtype  
${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

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

find_package(OpenCV REQUIRED)

# OMcntl
add_executable(OMcntl ./OMcntl/src/OMcntl.cpp ./OMcntl/src/ModbusRTU.cpp ./utility/src/utility.cpp)
target_link_libraries(OMcntl m ssm)

# config
add_executable(config ./config/src/config.cpp ./config/src/commonParam.cpp ./utility/src/utility.cpp)
target_link_libraries(config m ssm)
add_executable(config-viewer ./config/src/config-viewer.cpp ./config/src/commonParam.cpp ./utility/src/utility.cpp)
target_link_libraries(config-viewer m ssm)

# createWP
add_executable(createWPfile ./createWP/src/createWPfile.cpp ./createWP/src/createWP.cpp ./createWP/src/GraphDrawerWP.cpp ./utility/src/utility.cpp)
target_link_libraries(createWPfile m ssm)

# detectObstacle
add_executable(detectObstacle ./detectObstacle/src/detectObstacle.cpp ./detectObstacle/src/obp.cpp ./utility/src/utility.cpp)
target_link_libraries(detectObstacle m ssm)
add_executable(detectObstacle-viewer ./detectObstacle/src/detectObstacle-viewer.cpp ./detectObstacle/src/GraphDrawerOBP.cpp ./utility/src/utility.cpp)
target_link_libraries(detectObstacle-viewer m ssm)

# gnss-f9p
add_executable(gnss-f9p-handler ./gnss-f9p/src/gnss-f9p-handler.cpp ./gnss-f9p/src/rtknavi-f9p.cpp ./utility/src/utility.cpp)
target_link_libraries(gnss-f9p-handler m ssm)
add_executable(gnss-f9p-viewer ./gnss-f9p/src/gnss-f9p-viewer.cpp ./gnss-f9p/src/GraphDrawerGNSS-f9p.cpp  ./gnss-f9p/src/rtknavi-f9p.cpp ./utility/src/utility.cpp ./utility/src/log2txt.cpp)
target_link_libraries(gnss-f9p-viewer m ssm)

# imu_rt
add_executable(imu-handler ./imu_rt/src/imu-handler.cpp ./imu_rt/src/9axisIMU.cpp ./utility/src/utility.cpp)
target_link_libraries(imu-handler m ssm)
add_executable(imu-viewer ./imu_rt/src/imu-viewer.cpp ./imu_rt/src/GraphDrawerIMU.cpp ./utility/src/utility.cpp ./utility/src/log2txt.cpp)
target_link_libraries(imu-viewer m ssm)

# joystick
add_executable(manualOperation ./joystick/src/manualOperation.cpp ./OMcntl/src/ModbusRTU.cpp ./joystick/src/operateJoystick.cpp ./utility/src/utility.cpp)
target_link_libraries(manualOperation ssm rppjs)
add_executable(handle-viewer ./joystick/src/handle-viewer.cpp ./joystick/src/GraphDrawerHandle.cpp ./utility/src/utility.cpp ./utility/src/log2txt.cpp)
target_link_libraries(handle-viewer m ssm)
add_executable(accel-viewer ./joystick/src/accel-viewer.cpp ./joystick/src/GraphDrawerAccel.cpp ./utility/src/utility.cpp ./utility/src/log2txt.cpp)
target_link_libraries(accel-viewer m ssm)
add_executable(lever-viewer ./joystick/src/lever-viewer.cpp ./joystick/src/GraphDrawerLever.cpp ./utility/src/utility.cpp ./utility/src/log2txt.cpp)
target_link_libraries(lever-viewer m ssm)
add_executable(recordOperation ./joystick/src/recordOperation.cpp ./OMcntl/src/ModbusRTU.cpp ./joystick/src/operateJoystick.cpp ./utility/src/utility.cpp)
target_link_libraries(recordOperation ssm)

# localizer
add_executable(localizer ./localizer/src/localizer.cpp ./localizer/src/estimatePose.cpp ./navigate/src/wp-mgr.cpp ./utility/src/utility.cpp)
target_link_libraries(localizer m ssm)
add_executable(localizer-viewer ./localizer/src/localizer-viewer.cpp ./localizer/src/GraphDrawerPose.cpp ./utility/src/utility.cpp ./utility/src/log2txt.cpp)
target_link_libraries(localizer-viewer m ssm)

# navigate
add_executable(navigate ./navigate/src/navigate.cpp ./navigate//src/system-mgr.cpp ./navigate//src/wp-mgr.cpp ./OMcntl/src/ModbusRTU.cpp ./utility/src/utility.cpp)
target_link_libraries(navigate m ssm)

# urg-handler
add_executable(urg-handler ./urg/src/urg-handler.cpp ./urg/src/lidar.cpp ./utility/src/utility.cpp)
target_link_libraries(urg-handler m ssm scip2awd)
add_executable(urg-viewer ./urg/src/urg-viewer.cpp ./urg/src/GraphDrawerURG.cpp ./utility/src/utility.cpp)
target_link_libraries(urg-viewer m ssm)
#add_executable(urg-3Dviewer ./urg/src/urg-3Dviewer.cpp ./urg/src/GraphDrawerPCD.cpp ./utility/src/utility.cpp)
#target_link_libraries(urg-3Dviewer m ssm ${PCL_LIBRARIES})

# viewer
add_executable(navi-viewer ./viewer/src/navi-viewer.cpp ./viewer/src/GraphDrawerTrajectory.cpp ./navigate/src/wp-mgr.cpp ./utility/src/log2txt.cpp ./utility/src/utility.cpp)
target_link_libraries(navi-viewer m ssm)
add_executable(control-log2txt ./viewer/src/control-log2txt.cpp ./utility/src/log2txt.cpp ./utility/src/utility.cpp)
target_link_libraries(control-log2txt m ssm)
add_executable(WP-viewer ./viewer/src/WP-viewer.cpp ./viewer/src/GraphDrawerWP.cpp ./navigate/src/wp-mgr.cpp ./utility/src/utility.cpp)
target_link_libraries(WP-viewer m ssm)
add_executable(vel-viewer ./viewer/src/vel-viewer.cpp ./viewer/src/GraphDrawerVelocity.cpp ./utility/src/utility.cpp  ./utility/src/log2txt.cpp)
target_link_libraries(vel-viewer m ssm)




