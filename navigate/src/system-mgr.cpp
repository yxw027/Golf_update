/*
 * 芝刈機HerbF用System Manager
 * Date   : 2018.05.27
 * Update : 2019.08.18
 * Update : 2019.09.10
 * Update : 2019.11.18
 * Update : 2021.03.27
 * Author : T.Hasegawa
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>

#include <iostream>
#include <vector>
#include <limits>
#include <fstream>

#include <ssmtype/spur-odometry.h>
#include <ssm.hpp>
#include "system-mgr.hpp"
#include "config.hpp"
#include "utility.hpp"
#include "framework.hpp"

void SystemMgr::initilize( const config_property c, char *device_name, bool a_flag, unsigned int id, ROBOT_STATUS s )
{
	flag_analysis = a_flag;
	conf = c;
	robot_status = s;
	
	// 緊急停止モードの設定
	if( flag_analysis ){
		flag_emergency = false;		// 解析モードのときは緊急停止モード解除
	} else {
		flag_emergency = true;		// 自律走行開始時は緊急停止モード
		modbus.setConfig( c );
	}
	for( int i = 0 ; i < 2 ; i++ ){
		conf.cntl.l_K1[ i ] = DEG2RAD( c.cntl.l_K1[ i ] );
	}
	conf.cntl.clip_tgtAngle = DEG2RAD( c.cntl.clip_tgtAngle );
	conf.cntl.clip_tgtSteering = DEG2RAD( c.cntl.clip_tgtSteering );
	conf.wp_info.TR = conf.robot_info.wheelbase / tan( conf.cntl.clip_tgtSteering );
	printf( "%8.3f # TURNING RADIUS [m] OF A MOBILE ROBOT\n", conf.wp_info.TR );	// 確認用

	wp_mgr.initilize( conf, id, s );
#ifdef FOLLOW_LINE_REAL_TIMMING
	last_wp_cotrol = wp_mgr.getWPcurrent( );	// 一応、初期化
#else
#endif

	if( !flag_analysis ){
		modbus.openController( device_name );
		
		Gprint( "Set Zero Position for Handle\n" );
		modbus.setZeroPosition( _HANDLE );
		// 動作開始
		modbus.sendStart( _ACCEL );
		modbus.sendStart( _HANDLE );
		modbus.sendStart( _LEVER );
		// ニュートラル位置へ移動
		modbus.sendPos( _ACCEL, _ACCEL_POS_NEUTRAL );
		modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );
	}
}
void SystemMgr::resetParameter( void )
{
	debug.follow.curAng = 0;
	debug.follow.curTorque = 0;
	debug.follow.distance = 0;
	debug.follow.inpAng = 0;
	debug.follow.tgtAngVel = 0;
	debug.follow.tgtHandleAng = 0;
	debug.follow.tgtHandleAngVel = 0;
	debug.follow.tgtHandleAng_ex = 0;
	debug.follow.tgtSteeringAng = 0;
	debug.follow.tgtTheta = 0;
	debug.cVel.calStroke = 0;
	debug.cVel.curPos = 0;
	debug.cVel.inpPos = 0;
	debug.offset.estHandleAng = 0;
	debug.offset.estSteeringAng = 0;
	debug.offset.steering_offset = 0;

	motor_save.accel.input = 0;
	motor_save.accel.pos = 0;
	motor_save.accel.torque = 0;
	motor_save.handle.ang = 0;
	motor_save.handle.angvel = 0;
	motor_save.handle.input = 0;
	motor_save.handle.target = 0;
	motor_save.handle.target_angvel = 0;
	motor_save.handle.target_ex = 0;
	motor_save.handle.torque = 0;
	motor_save.lever.input = 0;
	motor_save.lever.pos = 0;
	motor_save.lever.torque = 0;
}
// ********************* getWPの関連 *************************
wp_gl SystemMgr::getWP( void )	// 通常のWP
{
	wp_current = wp_mgr.getWPcurrent( );
	wp_mgr.printWPcurrent( );
	return wp_current;
}
// 先読みWPとポジショニング用WP
wp_gl SystemMgr::getWP( localizer *odm )
{
	wp_control = wp_mgr.getWP( odm );
	wp_current = wp_mgr.getWPcurrent( );
	wp_mgr.printWP( &wp_control );
	if( robot_status == POSITIONING ){
		robot_status = wp_mgr.getRobotStatus( );	// POSITIONINGからNAVIへの移行
	}
	return wp_control;
}
// ********************* ゴール判定 *************************
bool SystemMgr::chkGoal( void )
{
	bool ret = wp_mgr.chkGoal( );
	if( ret ){
		if( !flag_analysis ){
			// アクセルをニュートラルへ戻す
			modbus.sendPos( _ACCEL, _ACCEL_POS_NEUTRAL );
			// 芝刈部の確認。必要であれば、戻す。
			if( wp_mgr.getOrderCuttingUnit( 0 ) == Up_CuttingUnit ){	// 上げる
				// 芝刈部上げる
				modbus.sendPos( _LEVER, _LEVER_POS_MIN );
				usleepSSM( conf.cntl.cuttingUnit_Up_time * 1000 * 1000 );
				// ニュートラル位置へ戻す
				modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );
			} else {
				modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );
			}
		}
	}
	return ret;
}
// ********************* ライン追従制御コマンド **********************
static int wait_time = 5;	// 5ms
bool SystemMgr::followLine( localizer *odm, double t )
{
	if( flag_emergency ){
		stopVehicle( );
		return true;
	}

	if( ( robot_status == NAVI ) || ( robot_status == POSITIONING ) ){
		My_Spur_line_GL( wp_control, odm, t );
		
	} else {
		wp_mgr.setFlagUpdateWP( false );	// WPを更新しない
		return true;
	}

	// Check whether the robot rearch at the WP.
	if( wp_mgr.chkOverLine( odm ) ){
		if( robot_status == NAVI ) wp_mgr.CountUpWP( );
		wp_mgr.setFlagUpdateWP( true );
		return false;
		
	} else {
		usleepSSM( wait_time * 1000 );
	}
	
	wp_mgr.setFlagUpdateWP( false );
	return true;
}
// ライン追従制御の本体
static double tgtSteeringAng = 0;	// 目標ステアリング角
static double estSteeringAng = 0;	// 現在速度と角速度から推定したステアリング角
static double steering_offset = 0;
// 何秒後の予測ハンドル角用変数
static double time_old = 0;
static double tgtHandleAng_old = 0;
static bool flag_first_loop = true;
void SystemMgr::My_Spur_line_GL( wp_gl wp, localizer *odm, double t )
{
	int gid = wp.gain_id;

#ifdef FOLLOW_LINE_REAL_TIMMING
	if( ( SIGN( wp.v ) > 0 ) && ( odm->dir == false ) ){	// 速度命令に対して動作が間に合っていない場合（慣性）
		wp = last_wp_cotrol;
	} else if( ( SIGN( wp.v ) < 0 ) && ( odm->dir == true ) ){	// 速度命令に対して動作が間に合っていない場合（慣性）
		wp = last_wp_cotrol;
	} else {
		last_wp_cotrol = wp;
	}
#else
#endif
	
// ---------------- 目標ステアリング角を計算 --------------->
	// 直線との距離を計算（直線より左側だと負、右側だと正（反時計回転が正））
	double distance = ( wp.x - odm->estPose.x ) * sin( -1.0*wp.theta ) + ( wp.y - odm->estPose.y ) * cos( -1.0*wp.theta );
	// 直線に対する目標侵入角度を計算 //（単位：rad）
	double tgtTheta = distance * conf.cntl.l_K1[ gid ];
	// 目標侵入角度をクリップ
	if( fabs( tgtTheta ) > conf.cntl.clip_tgtAngle ) tgtTheta = SIGN( tgtTheta ) * conf.cntl.clip_tgtAngle;
	// 世界座標系における車両の目標進行方位 // （単位：角度）
	tgtTheta = trans_q( tgtTheta + wp.theta );

	// 現在の車両の進行方位から見た目標進行方位との差分を計算し，時定数を考慮した上で目標角速度を求める（最大角速度でクリップ） // （単位：rad/s）
	double tgtAngVel;
	if( wp.v >= 0 ){	// 前進走行の場合
		tgtAngVel = conf.cntl.l_K2[ gid ] * trans_q( tgtTheta - odm->estPose.theta );
	} else {	// バック走行の場合
		tgtAngVel = conf.cntl.l_K2[ gid ] * trans_q( tgtTheta - odm->estPose.theta + M_PI );
	}
	
	// 目標角速度のクリップ
//	if( fabs( tgtAngVel ) > conf.navi.ang_vel ) tgtAngVel = SIGN( tgtAngVel ) * conf.navi.ang_vel;

	if( odm->estPose.v <= 0.05 ){
#ifdef CUTTING_ANGLE_ZERO
	// 目標ステアリング角を求める。車両速度が0.1m/s以下のとき目標ステアリング角はゼロ// （単位：rad）
		tgtSteeringAng = 0;
#else
	// 目標ステアリング角を求める。車両速度が0.1m/s以下のとき目標ステアリング角は変化なし// （単位：rad）
//		tgtSteeringAng = 0;
#endif
	} else {
		tgtSteeringAng = SIGN( wp.v ) * atan( conf.robot_info.wheelbase * tgtAngVel / odm->estPose.v );
		// （取り合えず、20degでクリップ）
		if( fabs( tgtSteeringAng ) > conf.cntl.clip_tgtSteering ) tgtSteeringAng = SIGN( tgtSteeringAng ) * conf.cntl.clip_tgtSteering;
	}
// <-------------------------------------------

//---------------- 現在のステアリング角とハンドル角を推定（handle_offsetの推定のため） -------> 
	//車両の現在速度と角速度から、現在のステアリング角を推定 
	if( odm->estPose.v <= 0.05 ){
//		estSteeringAng = 0;
	} else {
		estSteeringAng = SIGN( wp.v ) * atan( conf.robot_info.wheelbase * odm->estPose.w / odm->estPose.v );
	}
	//*********** 推定ステアリング角度を、取り合えず、40degでクリップ（これ正しい？？）*******************************************************
//	if( fabs( estSteeringAng ) > DEG2RAD( 40 ) ) estSteeringAng = SIGN( estSteeringAng ) * DEG2RAD( 40 );

	// 推定したステアリング角から、現在のハンドル角を推定
	double estHandleAng = RAD2DEG( estSteeringAng ) * conf.cntl.s_ratio + steering_offset;	// RadからDegへ変換（モーター入出力値の単位がDeg）
// <------------------------
	
	double curAng = 0;		// 現在ハンドル角（絶対角）
// ----------------- 現在のハンドル角を計測 --------------------->
	// 解析モード以外は、直接、モータドライバから取得
	if( !flag_analysis ){
		curAng = modbus.getAngle( );
	}

// <-------------------------------

	// ステアリング・オフセットの推定
	steering_offset += ( conf.cntl.beta * ( curAng - estHandleAng ) );

	// オフセットを補正後に、推定ハンドル角を再計算する。
	estHandleAng = RAD2DEG( estSteeringAng ) * conf.cntl.s_ratio + steering_offset;

	// 目標ハンドル角を求める。
	double tgtHandleAng = RAD2DEG( tgtSteeringAng ) * conf.cntl.s_ratio + steering_offset;
	
	// 何秒後かの目標値の予測
	double tgtHandleAng_ex;
	// 目標値の角速度 [degree/s]
	double tgtHandleAngVel;
	if( flag_first_loop ){
		flag_first_loop = false;
		time_old = t;
		tgtHandleAng_old = tgtHandleAng;
		tgtHandleAng_ex = tgtHandleAng;
	} else {
		// 時間差分 [s]
		double dT = t - time_old;
		//	printf( "dT=%f\n", dT );
		time_old = t;
		// 目標値の角速度 [degree/s]
		tgtHandleAngVel = ( tgtHandleAng - tgtHandleAng_old ) / dT;
		tgtHandleAng_old = tgtHandleAng;
		// 何秒後かの目標値の予測
		tgtHandleAng_ex = tgtHandleAng + tgtHandleAngVel * 0.1;	// 0.1秒後
	}
	
// ---------------------------------->
	
	debug.follow.distance = distance;
	debug.follow.tgtTheta = tgtTheta;
	debug.follow.tgtAngVel = tgtAngVel;
	debug.follow.tgtSteeringAng = tgtSteeringAng;
	debug.follow.tgtHandleAng = tgtHandleAng;
//	debug.follow.inpAng = inAng;
//	debug.follow.curAng = curAng;
//	debug.follow.curTorque = tau;
	debug.follow.tgtHandleAngVel = tgtHandleAngVel;
	debug.follow.tgtHandleAng_ex = tgtHandleAng_ex;
	debug.offset.estSteeringAng = estSteeringAng;
	debug.offset.estHandleAng = estHandleAng;
	debug.offset.steering_offset = steering_offset;

// <---------------------------------------
}
#ifdef VELOCITY_CONTROL_INTEGRATION_VERSION
// フィードフォワード用ストローク量の算出
static double calTargetStroke( double vel )
{
//	double ratio = 0.8;		// 算出量のratio割で出力(0 < ratio < 1)
	double len;

#ifdef SPEED_3000RPM	// 3000rpm
	len = 15.93 * vel + 32.59;
#else	// 1000〜1500rpm
	len = 37.40 * vel + 31.97;
#endif
	//if( vel >= 0 ) len *= ratio;
	//else {
		//double dl = len * ( 1.0 - ratio );
		//len += dl;
	//}
//fprintf(stderr,"len=%f\n",len);
	if( len > _ACCEL_POS_MAX )	len = _ACCEL_POS_MAX;
	if( len < _ACCEL_POS_MIN )	len = _ACCEL_POS_MIN;

	return( len );
}
#else
// フィードフォワード用ストローク量の算出
static double calTargetStroke( double vel )
{
	double len;

	if( vel == 0.0 ){
		return _ACCEL_POS_NEUTRAL;

	} else if( vel < 0.0 ){
#ifdef SPEED_3000RPM	// 3000rpm
		len = 10.31 * vel + 26.34;
#else	// 1000〜1500rpm
		len = 22.222 * vel + 25;
#endif
		if( len > _ACCEL_POS_NEUTRAL )	len = _ACCEL_POS_NEUTRAL;
		if( len < _ACCEL_POS_MIN )		len = _ACCEL_POS_MIN;
		
	} else if( vel > 0.0 ){

#ifdef SPEED_3000RPM	// 3000rpm
		len = 9.7544 * vel + 41.791;
#else	// 1000〜1500rpm
		len = 21.834 * vel + 42.271;
#endif
//		fprintf( stderr, "len=%f\n", len );
		
		if( len < _ACCEL_POS_NEUTRAL )	len = _ACCEL_POS_NEUTRAL;
		if( len > _ACCEL_POS_MAX )		len = _ACCEL_POS_MAX;
	}
	return( len );
}
#endif
static int WPid = -1;
static double time_old_velcntl = 0;
static double dvel_current = 0;
double SystemMgr::getTargetVelocity( double t )
{
	double vel = wp_current.v;

#ifdef SET_DESIRED_VELOCITY_PROFILE
	if( WPid != wp_current.id ){
		WPid = wp_current.id;
		time_old_velcntl = t;
//		dvel_current = wp_mgr.getVel( -1 );
	}
	double diff_v = vel - dvel_current;
//	fprintf( stderr, "[%d]v=%f, v1=%f\n", wp_current.id, vel, dvel_current );
	if( diff_v == 0 ){
		return vel;
	} else if( diff_v > 0 ){
		dvel_current += conf.navi.acc * ( t - time_old_velcntl );
		if( dvel_current >= vel ) dvel_current = vel;
	} else if( diff_v < 0 ){
		dvel_current -= conf.navi.acc * ( t - time_old_velcntl );
		if( dvel_current <= vel ) dvel_current = vel;
	}
	time_old_velcntl = t;
	return dvel_current;
#else
	return vel;
#endif
}
static double v_integ = 0.0;		// 積分成分
bool SystemMgr::ControlVelocity( localizer *odm, double t )
{
	if( flag_emergency ){
		stopVehicle( );
		return false;
	}
	
	// 目標速度の設定
	double d_vel;

	if( robot_status == POSITIONING ){	// 位置合わせ時の速度設定
		d_vel = wp_control.v;
	} else { // NAVI時の速度設定
		d_vel = getTargetVelocity( t );
	}

//	fprintf( stderr, "dv=%f\n", d_vel );

	// 速度制御
#ifdef VELOCITY_CONTROL_INTEGRATION_VERSION
	double calStroke = calTargetStroke( d_vel );
	double dv = d_vel - SIGN( d_vel ) * odm->estPose.v;
	double ratio = dv / fabs( d_vel );
	v_integ += ( conf.cntl.v_K2 * ratio );
	double inpPos = calStroke + ( conf.cntl.v_K1 * dv ) + v_integ;

	if( inpPos > _ACCEL_POS_MAX )		inpPos = _ACCEL_POS_MAX;
	if( inpPos < _ACCEL_POS_MIN )		inpPos = _ACCEL_POS_MIN;
#else
	double inpPos;
	double calStroke = calTargetStroke( d_vel );
//	fprintf( stderr, "calStroke=%f, \n",calStroke );

	if( d_vel >= 0 ){
		// 前進走行// 速度制御
		double dv = d_vel - odm->estPose.v;
//		fprintf( stderr, "dv=%f\n", dv );
//		if( fabs( d_vel ) == 0 ) d_vel = SIGN( d_vel )*0.1;
		double ratio = dv / fabs( d_vel );
//		fprintf( stderr, "ratio=%f, v_integ=%f\n", ratio, v_integ );
		v_integ += ( conf.cntl.v_K2 * ratio );
//		fprintf( stderr, "v_integ=%f, conf.cntl.v_K2=%f\n", v_integ, conf.cntl.v_K2 );
		inpPos = calStroke + ( conf.cntl.v_K1 * dv ) + v_integ;
//		fprintf( stderr, "inpPos=%f\n", inpPos );
		if( inpPos < _ACCEL_POS_NEUTRAL )	inpPos = _ACCEL_POS_NEUTRAL;
		if( inpPos > _ACCEL_POS_MAX )		inpPos = _ACCEL_POS_MAX;

	} else {
		// 後退走行// 速度制御
		double dv = d_vel + odm->estPose.v;		// odm->estPose.vは常にプラスのため＋の符号
		double ratio = dv / fabs( d_vel );
		v_integ += ( conf.cntl.v_K2 * ratio );
		inpPos = calStroke + ( conf.cntl.v_K1 * dv ) + v_integ;

		if( inpPos > _ACCEL_POS_NEUTRAL )	inpPos = _ACCEL_POS_NEUTRAL;
		if( inpPos < _ACCEL_POS_MIN )		inpPos = _ACCEL_POS_MIN;
	}
//	fprintf(stderr,"inpPos=%f\n",inpPos);
#endif
	if( flag_analysis ){
		if( odm->estPose.v < 0.1 ) inpPos = _ACCEL_POS_NEUTRAL;// 解析モード時は、速度0.1m/s以下ではストローク量_ACCEL_POS_NEUTRAL一定
	}

	motor_save.accel.input = inpPos;	
	debug.cVel.calStroke = calStroke;
	debug.cVel.d_vel = d_vel;
	debug.cVel.vel = odm->estPose.v;

	bool ret = sendAccelOrder( );
	return ret;
}
bool SystemMgr::sendAccelOrder( void )
{
	if( !flag_analysis ){
//************* アクセル用 ****************
		modbus.sendPos( _ACCEL, motor_save.accel.input );
		debug.cVel.inpPos = motor_save.accel.input;
		// 現在ストローク量の保存
		double curPos = modbus.getPosition( _ACCEL );
		debug.cVel.curPos = curPos;
		motor_save.accel.pos = curPos;
		// 現在トルクの割合を取得
		double tau_a = modbus.getTorque( _ACCEL );
		motor_save.accel.torque = tau_a;
		// アラーム・インフォメーション関係
		motor_save.accel.alarm = modbus.getAlarmCode( _ACCEL );
		motor_save.accel.info = modbus.getInfoCode( _ACCEL );

	} else {
		return false;
	}
	return true;
}

//static bool flag_handle_angle_limit = false;
static double recordAng = 0;
bool SystemMgr::sendHandleOrder( void )
{
	if( flag_emergency ){
		stopVehicle( );
		return false;
	}
	
	if( !flag_analysis ){

		double curAng = modbus.getAngle( );	// 現在角度の取得
		double tau_s = modbus.getTorque( _HANDLE );	// 現在トルクの割合を取得 
		double inAng =  debug.follow.tgtHandleAng_ex - curAng;

		// インフォメーション発行かトルク800以上の場合、それ以上回転しないように処理する
		if( modbus.getInfoCode( _HANDLE ) == 2 ){
			modbus.clearPositionDeviation( _HANDLE );
			flag_handle_angle_limit = true;
			recordAng = curAng;
			inAng = -1.0 * SIGN( tau_s ) * conf.motor.invHandleInput;
			modbus.resetPositionDeviation( _HANDLE );
		} else if( fabs( tau_s ) > conf.motor.thrHandleTorque ){
			flag_handle_angle_limit = true;
			recordAng = curAng;
			inAng = -1.0 * SIGN( tau_s ) * conf.motor.invHandleInput;
		}
		if( flag_handle_angle_limit ){
			if( fabs( tau_s ) > conf.motor.thrHandleTorque ){
				inAng = -1.0 * SIGN( tau_s ) * conf.motor.invHandleInput;
			} else {
				flag_handle_angle_limit = false;
			}
		}

		// 中空アクチュエータへ回転命令
		modbus.sendAng( _HANDLE, inAng );

		debug.follow.inpAng = inAng;
		motor_save.handle.input = inAng;
		// 現在角度の取得
		curAng = modbus.getAngle( );
		debug.follow.curAng = curAng;
		motor_save.handle.ang = curAng;
		// 現在トルクの割合を取得 
		tau_s = modbus.getTorque( _HANDLE );
		debug.follow.curTorque = tau_s;
		motor_save.handle.torque = tau_s;
		// 目標ハンドル角関係
		motor_save.handle.target = debug.follow.tgtHandleAng;
		motor_save.handle.target_ex = debug.follow.tgtHandleAng_ex;
		motor_save.handle.target_angvel = debug.follow.tgtHandleAngVel;
		// アラーム・インフォメーション関係
		motor_save.handle.alarm = modbus.getAlarmCode( _HANDLE );
		motor_save.handle.info = modbus.getInfoCode( _HANDLE );
		
	} else {
		return false;
	}
	return true;
}

// ********************* Cutting Unitの関連 *************************
static unsigned int mode_CuttingUnit = 0; // 昇:2, 降:1, 戻:0
static double startTime_CuttingUnit = 0;
bool SystemMgr::setCuttingUnit( void )
{
	if( flag_analysis ) return false;
	if( mode_CuttingUnit != Neutral_CuttingUnit )	return false;	// 要確認（ニュートラルの時のみ命令可）
	
	if( !chkUpdateWP( ) && wp_mgr.getOrderCuttingUnit( -1 ) == Down_CuttingUnit ){	// 下ろす
		modbus.sendPos( _LEVER, _LEVER_POS_MAX );
		mode_CuttingUnit = Down_CuttingUnit;
		startTime_CuttingUnit = gettimeSSM( );
		return false;
		
	} else if( !chkUpdateWP( ) && wp_mgr.getOrderCuttingUnit( -1 ) == Up_CuttingUnit ){	// 上げる
		modbus.sendPos( _LEVER, _LEVER_POS_MIN );
		startTime_CuttingUnit = gettimeSSM( );
		mode_CuttingUnit = Up_CuttingUnit;
		return false;	
	}

	return true;
}
bool SystemMgr::returnCuttingUnit( void )
{
	if( flag_analysis ) return false;
	if( mode_CuttingUnit == Neutral_CuttingUnit )	return false;	// 要確認（ニュートラル以外の時のみ命令可）

	unsigned int time_now = ( int )( gettimeSSM( ) - startTime_CuttingUnit );
	if( mode_CuttingUnit == Down_CuttingUnit ){
		if( time_now >= conf.cntl.cuttingUnit_Down_time ){
			// ニュートラル位置へ戻す
			modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );
			startTime_CuttingUnit = 0;
			mode_CuttingUnit = Neutral_CuttingUnit;
		
			return false;
		}
	} else if( mode_CuttingUnit == Up_CuttingUnit ){
		if( time_now >= conf.cntl.cuttingUnit_Up_time ){
			// ニュートラル位置へ戻す
			modbus.sendPos( _LEVER, _LEVER_POS_NEUTRAL );
			startTime_CuttingUnit = 0;
			mode_CuttingUnit = Neutral_CuttingUnit;
		
			return false;
		}
	}
	return true;
}

// ********************* 緊急停止 **********************
static unsigned long status_botton = 0;
static bool flag_startVehicle_useJoystick = false;
void SystemMgr::chkEmergency4Joystick( ssm::JSData *joy )
{
	if( ( ( joy->button & ssm::JS_BUTTON_0 ) == ssm::JS_BUTTON_0 ) && 
		( status_botton != ssm::JS_BUTTON_0 ) ){	// ssm::JS_BUTTON_0 -> SELECT
		printf("Stop\n");
		flag_emergency = true;
		status_botton = ssm::JS_BUTTON_0;
		stopVehicle( );
		flag_startVehicle_useJoystick = false;
					
	} else if( ( ( joy->button & ssm::JS_BUTTON_3 ) == ssm::JS_BUTTON_3 ) && 
		( status_botton != ssm::JS_BUTTON_3 ) ){	// ssm::JS_BUTTON_3 -> START
		printf("Start\n");
		flag_emergency = false;
		status_botton = ssm::JS_BUTTON_3;
		flag_startVehicle_useJoystick = true;
		
	} else {
//		status_botton = 0;
	}
}
// ********************* 衝突回避 **********************
bool SystemMgr::collisionAvoidance( obp_fs *p )
{
	if( !flag_startVehicle_useJoystick ) return false;

	if( p->status == STOP ){
		printf("Stop\n");
		flag_emergency = true;
		stopVehicle( );
					
	} else if( p->status == DECELERATION ){
		printf("DECELERATION\n");
		flag_emergency = false;
		
	} else if( p->status == TRAVELING ){
		printf("TRAVELING\n");
		flag_emergency = false;
	}
	return true;
}
// ********************* 停止コマンド **********************
void SystemMgr::stopVehicle( void )
{
	if( !flag_analysis ){
		// ニュートラルに戻しているのみ。
		// 坂道で停まれない課題あり。
		modbus.sendPos( _ACCEL, _ACCEL_POS_NEUTRAL );	// ニュートラル位置を入力
	}
}
