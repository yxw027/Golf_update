#ifndef FRAMEWORK_HPP
#define FRAMEWORK_HPP

#include "ModbusRTU.hpp"
#include "9axisIMU.hpp"
#include "wp-mgr.hpp"

//******* ModbusRTU *********
//#define ModbusRTU_CLASS	ModbusRTU_DirectOrder
#define ModbusRTU_CLASS	ModbusRTU_IndirectOrder

//******* IMU *********
//#define IMU_CLASS	RT_9axisIMU
#define IMU_CLASS	RT_9axisIMU_ACII
//#define IMU_CLASS	RT_9axisIMU_ACII_SIMPLE

//******* WP *********
//#define WP_MGR_CLASS	WpMgr_Base
//#define WP_MGR_CLASS	WpMgr_Positioning
#define WP_MGR_CLASS	WpMgr_Proactive

//******* detect Obstacle ******
//#define detectObstacle_CLASS detectObstacle_2D
#define detectObstacle_CLASS detectObstacle_3D

//******* Navi Viewer *********
//#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory
#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory_Proactive



//+++++++ 回転数の設定 +++++++++
#define SPEED_3000RPM


#endif
