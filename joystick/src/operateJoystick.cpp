#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include <ssm.hpp>
#include <joystick.hpp>
#include "operateJoystick.hpp"
#include "OMcntl.hpp"
#include "utility.hpp"

void operateJoystick::openOMController( const char *dev )
{
	modbus.openController( dev );
}
static int keyframeSkip = 20;		// キーフレーム間隔
static unsigned long counter = 0;
static bool flag_limit = false;
static double recordAng = 0;
void operateJoystick::Handle( ssm::JSData *joy2 )
{
	//	2:右、左右(左-1、右1)
	double rotation = joy2->axis[ 2 ];
	double scale2 = 15;		// if rotation=1 then rotate every 50 degree
	double inAng = rotation * scale2;
	
	double curAng = modbus.getAngle( );	// 現在角度の取得
	double tau = modbus.getTorque( _HANDLE );	// 現在トルクの割合を取得 

	// ハンドルロック時の回避処理
	if( modbus.getInfoCode( _HANDLE ) == 2 ){
		modbus.clearPositionDeviation( _HANDLE );
		flag_limit = true;
		recordAng = curAng;
		inAng = -1.0 * SIGN( tau ) * conf.motor.invHandleInput;
		modbus.resetPositionDeviation( _HANDLE );
	} else if( fabs( tau ) > conf.motor.thrHandleTorque ){
		flag_limit = true;
		recordAng = curAng;
		inAng = -1.0 * SIGN( tau ) * conf.motor.invHandleInput;
	}
	if( flag_limit ){
		if( fabs( tau ) > conf.motor.thrHandleTorque ){
			inAng = -1.0 * SIGN( tau ) * conf.motor.invHandleInput;
		} else {
			flag_limit = false;
		}
	}

	// 中空アクチュエータへ回転命令
	modbus.sendAng( _HANDLE, inAng );
	curAng = modbus.getAngle( );	// 現在角度の取得
	tau = modbus.getTorque( _HANDLE );	// 現在トルクの割合を取得 
	
	motorInfo.handle.input = inAng;
	motorInfo.handle.ang = curAng;
	motorInfo.handle.torque = tau;
	// アラーム・インフォメーション関係
	motorInfo.handle.alarm = modbus.getAlarmCode( _HANDLE );
	motorInfo.handle.info = modbus.getInfoCode( _HANDLE );

	if( counter % keyframeSkip == 0 ){			// キーフレームのときだけ表示
		printf( "ang=%3.2f[deg]", curAng );
		printf( ", tau=%f\n", tau );
		if( counter >= 30000 ) counter = 0;
	}
	counter++;
}

void operateJoystick::Accel( ssm::JSData *joy2 )
{
	//	1:左、上下(上-1、下1)
	double linear = joy2->axis[ 1 ];
	double neutral = ( double )_ACCEL_POS_NEUTRAL / 100.0;
	linear = -1.0 * ( _ACCEL_POS_MAX - neutral ) * linear + neutral;

	if( linear < 0 ) linear = 0; 
	double scale1 = 100;		// if linear=0〜1 then 0cm〜10cm
	double inPos = linear * scale1;

	if( inPos < _ACCEL_POS_MIN) inPos = _ACCEL_POS_MIN;
	if( inPos > _ACCEL_POS_MAX ) inPos = _ACCEL_POS_MAX;
	modbus.sendPos( _ACCEL, inPos );

	double curPos = modbus.getPosition( _ACCEL );//( double )step / 1000;
	if( counter % keyframeSkip == 0 ){			// キーフレームのときだけ行う
		printf( "pos=%3.2f[cm], ", curPos );
	}

	motorInfo.accel.input = inPos;
	motorInfo.accel.pos = curPos;
	motorInfo.accel.torque = modbus.getTorque( _ACCEL );
	// アラーム・インフォメーション関係
	motorInfo.accel.alarm = modbus.getAlarmCode( _ACCEL );
	motorInfo.accel.info = modbus.getInfoCode( _ACCEL );

}
static unsigned int mode_CuttingUnit = 0; // 昇:2, 降:1, 戻:0
static double startTime_CuttingUnit = 0;
void operateJoystick::Lever( int mode )
{
	double inPos = 0;
	double curPos = 0;
	if( mode == 1 ){	// 下ろす
		modbus.sendPos( _LEVER, _LEVER_POS_MAX );
		mode_CuttingUnit = 1;
		startTime_CuttingUnit = gettimeSSM( );

	} else if( mode == 2 ){	// 上げる
		modbus.sendPos( _LEVER, _LEVER_POS_MIN );
		startTime_CuttingUnit = gettimeSSM( );
		mode_CuttingUnit = 2;
	}

}
bool operateJoystick::returnNeutral_Lever( void )
{
	if( mode_CuttingUnit == 0 )	return false;	// 要確認（ニュートラル以外の時のみ命令可）

	unsigned int time_now = ( int )( gettimeSSM( ) - startTime_CuttingUnit );
	if( mode_CuttingUnit == 1 ){
		if( time_now >= conf.cntl.cuttingUnit_Down_time ){
			modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );

			startTime_CuttingUnit = 0;
			mode_CuttingUnit = 0;
		
			return false;
		}
	} else if( mode_CuttingUnit == 2 ){
		if( time_now >= conf.cntl.cuttingUnit_Up_time ){
			modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );

			startTime_CuttingUnit = 0;
			mode_CuttingUnit = 0;
		
			return false;
		}
	}
	return true;
}
void operateJoystick::getMotorInfo( int id )
{
	if( id == _HANDLE ){
		double inAng = 0;
		modbus.sendAng( _HANDLE, inAng );		// 中空アクチュエータへ回転命令
		motorInfo.handle.input = inAng;
		motorInfo.handle.ang = modbus.getAngle( );	// 現在角度の取得
		motorInfo.handle.torque = modbus.getTorque( _HANDLE );	// 現在トルクの割合を取得
		// アラーム・インフォメーション関係
		motorInfo.handle.alarm = modbus.getAlarmCode( _HANDLE );
		motorInfo.handle.info = modbus.getInfoCode( _HANDLE );
	} else if( id == _LEVER ){
		double inPos = _LEVER_POS_NEUTRAL;
		modbus.sendPos( _LEVER, inPos );	// 現在位置を得るためにドライバへ命令を送信
		motorInfo.lever.input = inPos;
		motorInfo.lever.pos = modbus.getPosition( _LEVER );
		motorInfo.lever.torque = modbus.getTorque( _LEVER );
		// アラーム・インフォメーション関係
		motorInfo.lever.alarm = modbus.getAlarmCode( _LEVER );
		motorInfo.lever.info = modbus.getInfoCode( _LEVER );
	} else if( id == _ACCEL ){
		double inPos = _ACCEL_POS_NEUTRAL;
		modbus.sendPos( _ACCEL, inPos );
		motorInfo.accel.input = inPos;
		motorInfo.accel.pos = modbus.getPosition( _ACCEL );
		motorInfo.accel.torque = modbus.getTorque( _ACCEL );
		// アラーム・インフォメーション関係
		motorInfo.accel.alarm = modbus.getAlarmCode( _ACCEL );
		motorInfo.accel.info = modbus.getInfoCode( _ACCEL );
	}
}
bool operateJoystick::operateStick( ssm::JSData *joy1 )
{
	bool ret = false;
	if( ( status_botton == ssm::JS_BUTTON_3 ) || ( status_botton == ssm::JS_BUTTON_0 ) ){
		Accel( joy1 );
		Handle( joy1 );
		getMotorInfo( _LEVER );
		ret = true;
	}
	return ret;
}
void operateJoystick::Free( void )
{
	Gprint( "SELECT:Free\n" );
	status_botton = ssm::JS_BUTTON_0;
	modbus.sendFree( _ACCEL );
	modbus.sendFree( _HANDLE );
	modbus.sendFree( _LEVER );
}
void operateJoystick::Start( void )
{
	Gprint( "START:Start\n" );
	status_botton = ssm::JS_BUTTON_3;
	modbus.sendStart( _ACCEL );
	modbus.sendStart( _HANDLE );
	modbus.sendStart( _LEVER );
}
void operateJoystick::returnZero( void )
{
	Gprint( "PS:ReturnZero\n" );
	status_botton = ssm::JS_BUTTON_16;
	// ID=1
	modbus.returnZero( _ACCEL );		// 高速原点復帰命令
	modbus.resetZeroCommand( _ACCEL );	// 命令解除

	double threAng = 50;
	double curAng = modbus.getAngle( );	// 現在角度の取得
	double inputAngle = 50;
	
	// ID=2
	// 360度分の復帰
	if( curAng > 0 ){
		while( curAng > threAng ){
			// 中空アクチュエータへ回転命令
			modbus.sendAng( _HANDLE, -1.0*inputAngle );
			curAng = modbus.getAngle( );	// 現在角度の取得
			usleep( 10 * 1000 );
		}
	} else if( curAng < 0 ){
		while( curAng < -1.0*threAng ){
			// 中空アクチュエータへ回転命令
			modbus.sendAng( _HANDLE, inputAngle );
			curAng = modbus.getAngle( );	// 現在角度の取得
			usleep( 10 * 1000 );
		}
	}
	usleep( 100 * 1000 );
	// 高速ゼロ点復帰（ただし、±50度以内のみ有効）
	modbus.returnZero( _HANDLE );		// 高速原点復帰命令
	modbus.resetZeroCommand( _HANDLE );	// 命令解除
	modbus.resetAngle( );
	
	// ID=3
	modbus.returnZero( _LEVER );		// 高速原点復帰命令
	modbus.resetZeroCommand( _LEVER );	// 命令解除
}
void operateJoystick::resetALM( void )
{
	Gprint( "丸ボタン:ALM-Reset\n" );
	status_botton = ssm::JS_BUTTON_13;
	modbus.sendALM_RST( _ACCEL );
	modbus.sendALM_RST( _HANDLE );
	modbus.sendALM_RST( _LEVER );
}
void operateJoystick::setZeroPosition( void )
{
	Gprint( "Xボタン:Set Zero Position\n" );
	status_botton = ssm::JS_BUTTON_14;
	modbus.setZeroPosition( _HANDLE );
}
void operateJoystick::operateBotton( ssm::JSData *joy1 )
{
	returnNeutral_Lever( );
	
	if( ( ( joy1->button & ssm::JS_BUTTON_0 ) == ssm::JS_BUTTON_0 ) && 
		( status_botton != ssm::JS_BUTTON_0 ) ){	// ssm::JS_BUTTON_0 -> SELECT
		Free( );

	} else if( ( ( joy1->button & ssm::JS_BUTTON_3 ) == ssm::JS_BUTTON_3 ) && 
		( status_botton != ssm::JS_BUTTON_3 ) ){	// ssm::JS_BUTTON_3 -> START
		Start( );

	} else if( ( ( joy1->button & ssm::JS_BUTTON_16 ) == ssm::JS_BUTTON_16 ) && 
		( status_botton != ssm::JS_BUTTON_16 ) ){	//	ssm::JS_BUTTON_16 -> PS
		returnZero( );

	} else if( ( ( joy1->button & ssm::JS_BUTTON_13 ) == ssm::JS_BUTTON_13 ) && 
		( status_botton != ssm::JS_BUTTON_13 ) ){	//	ssm::JS_BUTTON_13 -> 丸ボタン
		resetALM( );

	} else if( ( ( joy1->button & ssm::JS_BUTTON_14 ) == ssm::JS_BUTTON_14 ) && 
		( status_botton != ssm::JS_BUTTON_14 ) ){	//	ssm::JS_BUTTON_14 -> ☓ボタン
		setZeroPosition( );	// ステアリングのみ				
		
	} else if( ( ( joy1->button & ssm::JS_BUTTON_12 ) == ssm::JS_BUTTON_12 ) && 
		( status_botton != ssm::JS_BUTTON_12 ) ){	//	ssm::JS_BUTTON_12 -> △ボタン
		Lever( 1 );	// カッティングユニットを下ろす

	} else if( ( ( joy1->button & ssm::JS_BUTTON_15 ) == ssm::JS_BUTTON_15 ) && 
		( status_botton != ssm::JS_BUTTON_15 ) ){	//	ssm::JS_BUTTON_15 -> □ボタン
		Lever( 2 );	// ッティングユニットを上げる
		
	} else {

	}
}
void operateJoystick::printProperty( ssm::JSProperty *joy )
{
	printf( "\n<JOYSTICK INFO>\n" );
	printf( "product  : %s\n", joy->name );
	printf( "version  : %d\n", joy->version );

	printf( "\n" );
	printf( "%8d # NUMBER OF AXIS\n", joy->axisNum );
	printf( "%8d # NUMBER OF BOTTON\n", joy->buttonNum );

}
