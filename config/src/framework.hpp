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
//#define USE_ELEVATION


//******* IMU ***************************************
#define IMU_CLASS	RT_9axisIMU
//#define IMU_CLASS	RT_9axisIMU_ACII
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
#define SystemMgr_CLASS SystemMgr

//+++++++ 回転数の設定 +++++++++
#define SPEED_3000RPM
//+++++++ 目標速度プロファイルを設定 +++++++++
//#define SET_DESIRED_VELOCITY_PROFILE 
//+++++++ 前後退の速度制御をまとめたプログラムを使用 +++++++++
#define VELOCITY_CONTROL_INTEGRATION_VERSION 
//+++++++ 低速時にタイヤ切れ角ゼロの設定 +++++++++
//#define CUTTING_ANGLE_ZERO
//+++++++ 前後退切替時に実際に動作が切り替わったタイミングを確認してFollowLine用のWPを切り替える設定 +++++++++
#define FOLLOW_LINE_REAL_TIMMING

//******* Navi Viewer *********************************
//#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory
#define NAVI_VIEWER_CLASS	GraphDrawerTrajectory_Proactive

#endif
