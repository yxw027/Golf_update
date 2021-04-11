#ifndef FRAMEWORK_HPP
#define FRAMEWORK_HPP

#include "ModbusRTU.hpp"
#include "9axisIMU.hpp"
#include "wp-mgr.hpp"

//******* ModbusRTU *********************************
//#define ModbusRTU_CLASS	ModbusRTU_DirectOrder
#define ModbusRTU_CLASS	ModbusRTU_IndirectOrder


//******* GNSS-F9P **********************************
//+++++++++ 楕円対高でなく標高を使用してENUへ変換後、Zを標高へ置き換え +++++++
#define USE_ELEVATION


//******* IMU ***************************************
//#define IMU_CLASS	RT_9axisIMU
#define IMU_CLASS	RT_9axisIMU_ACII
//#define IMU_CLASS	RT_9axisIMU_ACII_SIMPLE


//******* detect Obstacle ****************************
//#define detectObstacle_CLASS detectObstacle_2D
#define detectObstacle_CLASS detectObstacle_3D

//#define GraphDrawerOBP_CLASS GraphDrawerOBP_2D
#define GraphDrawerOBP_CLASS GraphDrawerOBP_3D


//******* WP ******************************************
//#define WP_MGR_CLASS	WpMgr_Base
//#define WP_MGR_CLASS	WpMgr_Positioning
#define WP_MGR_CLASS	WpMgr_Proactive


//******** Navigator *********************************
//+++++++ 回転数の設定 +++++++++
#define SPEED_3000RPM


//******* Navi Viewer *********************************
//#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory
#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory_Proactive

#endif
