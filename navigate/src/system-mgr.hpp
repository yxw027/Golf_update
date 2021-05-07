#ifndef SYSTEM_MGR_HPP
#define SYSTEM_MGR_HPP

#include <stdlib.h>
#include <joystick.hpp>
#include "config.hpp"
#include "localizer.hpp"
#include "control.hpp"
#include "wp.hpp"
#include "wp-mgr.hpp"
#include "detectObstacle.hpp"
#include "framework.hpp"

#define Up_CuttingUnit		2
#define Down_CuttingUnit	1
#define Neutral_CuttingUnit	0

class SystemMgr
{
protected:
	ModbusRTU_CLASS modbus;	// モータードライバと接続
	WP_MGR_CLASS wp_mgr;
	wp_gl wp_current;		// 車両代表点におけるWP
	wp_gl wp_control;		// ライン追従制御に使用するWP（先読みWP, ポジショニングWP）
#ifdef FOLLOW_LINE_REAL_TIMMING
	wp_gl last_wp_cotrol;	// 1つ前のライン追従制御用WP（前後退切替時に実際に動作が切り替わるまで使用）
#else
#endif

	enum ROBOT_STATUS robot_status;	// ロボットの状態
	config_property conf;
	
	bool flag_analysis;		// falseならSpurを動作させる
	bool flag_emergency;	// joystickによる緊急停止用フラグ
	
	bool chkOverLine( localizer *odm );
	void My_Spur_line_GL( wp_gl wp, localizer *odm, double t );
	
	control debug;	// 制御変数の確認用
	OMcntl motor_save;	// 解析モードでない場合は、３つのモータの情報をSSMへ登録する
	bool flag_handle_angle_limit;	// ハンドルがロックしている場合-->true

	void resetParameter( void );
	double getTargetVelocity( double t );	// 目標車両速度の算出

public:
	SystemMgr( void ) : flag_emergency( true ), flag_handle_angle_limit( false ) { }
	~SystemMgr( void ){ }

	void initilize( const config_property c, char *device_name, bool a_flag, unsigned int id, ROBOT_STATUS s );
	void closeAllProcess( void ){ if( !flag_analysis ) modbus.closeController( ); }
	
	wp_gl getWP( localizer *odm );	// 先読みWPとポジショニング用WP
	wp_gl getWP( void );	// 通常のWP
	void CountUpWP( void ){ wp_mgr.CountUpWP( ); }
	bool chkUpdateWP( void ){ return wp_mgr.chkUpdateWP( ); }
	bool chkGoal( void );
	
	void setRobotStatus( ROBOT_STATUS s ){ robot_status = s; }
	OMcntl getMotorData( void ){ return motor_save; }
	control getControlData( void ){ return debug; }
	
	virtual bool followLine( localizer *odm, double t );
	virtual bool ControlVelocity( localizer *odm, double t );

	void stopVehicle( void );
	bool setCuttingUnit( void );
	bool returnCuttingUnit( void );

	void chkEmergency4Joystick( ssm::JSData *joy );
	bool collisionAvoidance( obp_fs *p );
	bool sendHandleOrder( void );
	bool sendAccelOrder( void );
};

#endif
