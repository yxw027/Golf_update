#ifndef FRAMEWORK_HPP
#define FRAMEWORK_HPP

#include "ModbusRTU.hpp"
#include "9axisIMU.hpp"
//#include "wp-mgr.hpp"

//******* ModbusRTU *********************************
//#define ModbusRTU_CLASS	ModbusRTU_DirectOrder
#define ModbusRTU_CLASS	ModbusRTU_IndirectOrder


//******* GNSS-F9P **********************************
//+++++++++ 楕円対高でなく標高を使用してENUへ変換後、Zを標高へ置き換え +++++++
//#define USE_ELEVATION 1


//******* IMU ***************************************
#define IMU_CLASS	RT_9axisIMU
//#define IMU_CLASS	RT_9axisIMU_ACII
//#define IMU_CLASS	RT_9axisIMU_ACII_SIMPLE


//******* Localizer ***************************************
#define Use_transReferencePoint_3D 1


//******* detect Obstacle ****************************
//#define detectObstacle_CLASS detectObstacle_2D
#define detectObstacle_CLASS detectObstacle_3D

//#define GraphDrawerOBP_CLASS GraphDrawerOBP_2D
#define GraphDrawerOBP_CLASS GraphDrawerOBP_3D


//******* WP ******************************************
//#define WP_MGR_CLASS	WpMgr_Base
//#define WP_MGR_CLASS	WpMgr_Positioning
#define WP_MGR_CLASS	WpMgr_Proactive

#define Update_Proactive_WP_AnyTime 1

//******** Navigator *********************************
#define SystemMgr_CLASS SystemMgr

//+++++++ 回転数の設定 +++++++++
#define SPEED_3000RPM 1
//+++++++ 目標速度プロファイルを設定 +++++++++
//#define SET_DESIRED_VELOCITY_PROFILE 1
//+++++++ 前後退の速度制御をまとめたプログラムを使用 +++++++++
#define VELOCITY_CONTROL_INTEGRATION_VERSION 1
//+++++++ 低速時にタイヤ切れ角ゼロの設定 +++++++++
//#define CUTTING_ANGLE_ZERO 1
//+++++++ 前後退切替時に実際に動作が切り替わったタイミングを確認してFollowLine用のWPを切り替える設定 +++++++++
#define FOLLOW_LINE_REAL_TIMMING 1

//+++++++ 円弧追従を使用 +++++++++
#define Use_Follow_Circle	1

//******* Navi Viewer *********************************
//#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory
#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory_Proactive

#endif
