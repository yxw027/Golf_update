/*
 * Date : 2020.04.03
 * Update : 2021.03.21
 * Author : T.Hasegawa
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.hpp"
#include "commonParam.hpp"

int commonParam::initialize( char *path, bool print, const char* mode )
{
	flag_all_print = print;
	sprintf( flag_mode, "%s", mode );
	
	int len = strlen( flag_mode );
	if( strncmp( flag_mode, "r", len ) == 0 ){
		// 設定ファイルをオープン
		char filename[ STRLEN ];
		sprintf( filename, "%s/GM_Config.cfg", path );
		printf( "\nParameter File = %s\n\n", filename );	// 確認用
		fp_config = isValidFile( filename, "r", "cfg" );
		readProperty( fp_config, path );
	} else if( strncmp( flag_mode, "w", len ) == 0 ){
		// 保存ファイルの作成
		char filename[ STRLEN ];
		sprintf( filename, "%s/GM_Config_save.cfg", path );
		printf( "\nSave File = %s\n\n", filename );	// 確認用
		fp_config = isValidFile( filename, "w", "cfg" );
	} else {
		Gprint( "Error. Mismatch of File Mode\n" );
		exit( EXIT_FAILURE );
	}
	return EXIT_SUCCESS;
}
int commonParam::readProperty( FILE *fp, char *path )
{
//	printf( "\n<ROBOT INFO>\n" );
	property.robot_info.width = atof( getWord( fp ) );
	property.robot_info.length = atof( getWord( fp ) );
	property.robot_info.wheelbase = atof( getWord( fp ) );

//	printf( "\n<GNSS RECIEVER INFO>\n" );
	property.gnss.offset[ _X ] = atoi( getWord( fp ) );
	property.gnss.offset[ _Y ] = atoi( getWord( fp ) );
	property.gnss.offset[ _Z ] = atoi( getWord( fp ) );
	property.gnss.coordinate_id = atoi( getWord( fp ) );

//	printf( "\n<INITIAL POSITION>\n" );
	property.init_pose.x = atof( getWord( fp ) );
	property.init_pose.y = atof( getWord( fp ) );
	property.init_pose.theta = atof( getWord( fp ) );
	property.init_pose.v = 0;
	property.init_pose.w = 0;

//	printf( "\n<FUSION INFO>\n" );
	property.fusion.alpfa[ _X ] = atof( getWord( fp ) );
	property.fusion.alpfa[ _Y ] = atof( getWord( fp ) );
	property.fusion.alpfa[ _YAW ] = atof( getWord( fp ) );

//	printf( "\n<NAVIGATION INFO>\n" );
	property.navi.fvel = atof( getWord( fp ) );
	if( property.navi.fvel < 0.0 ){
		fprintf( stderr, "Error! property.navi.fvel is nagative.\n" );
		exit( EXIT_FAILURE );
	}
	property.navi.bvel = atof( getWord( fp ) );
	if( property.navi.bvel > 0.0 ){
		fprintf( stderr, "Error! property.navi.bvel is positive.\n" );
		exit( EXIT_FAILURE );
	}
	property.navi.ang_vel = atof( getWord( fp ) );
	property.navi.acc = atof( getWord( fp ) );
	property.navi.ang_acc = atof( getWord( fp ) );

//	printf( "\n<WP INFO>\n" );
	property.wp_info.TR = atof( getWord( fp ) );
	sprintf( property.wp_info.path, "%s", path );
	sprintf( WPfile, "%s", getWord( fp ) );
	sprintf( property.wp_info.filename, "%s/%s", path, WPfile );
	
//	printf( "\n<CONTROL INFO>\n" );
	property.cntl.beta = atof( getWord( fp ) );
	property.cntl.s_ratio = atof( getWord( fp ) );
	for( int i = 0 ; i < NUM_GAIN_TYPE ; i++ ){
		property.cntl.l_K1[ i ] = atof( getWord( fp ) );
	}
	for( int i = 0 ; i < NUM_GAIN_TYPE ; i++ ){
		property.cntl.l_K2[ i ] = atof( getWord( fp ) );
	}
	property.cntl.clip_tgtAngle = atof( getWord( fp ) );
	property.cntl.clip_tgtSteering = atof( getWord( fp ) );
	property.cntl.minStroke = atof( getWord( fp ) );
	property.cntl.maxStroke = atof( getWord( fp ) );
	property.cntl.v_K1 = atof( getWord( fp ) );
	property.cntl.v_K2 = atof( getWord( fp ) );
	property.cntl.cuttingUnit_Down_time = atoi( getWord( fp ) );
	property.cntl.cuttingUnit_Up_time = atoi( getWord( fp ) );

//	printf( "\n<MOTOR INFO>\n" );
	property.motor.thrHandleTorque = atof( getWord( fp ) );
	property.motor.invHandleInput = atof( getWord( fp ) );
	property.motor.diffHandleAngle = atof( getWord( fp ) );
	property.motor.h_angvel = atof( getWord( fp ) );
	property.motor.h_angacc = atof( getWord( fp ) );
	property.motor.a_vel = atof( getWord( fp ) );
	property.motor.a_acc = atof( getWord( fp ) );
		
//	printf( "\n<3D URG INFO>\n" );
	property.urg3d.rot[ _ROLL ] = atoi( getWord( fp ) );
	property.urg3d.rot[ _PITCH ] = atoi( getWord( fp ) );
	property.urg3d.rot[ _YAW ] = atoi( getWord( fp ) );
	property.urg3d.offset[ _X ] = atoi( getWord( fp ) );
	property.urg3d.offset[ _Y ] = atoi( getWord( fp ) );
	property.urg3d.offset[ _Z ] = atoi( getWord( fp ) );
	
//	printf( "\n<OBSTACLE DETECTION INFO>\n" );
	for( int i = 0 ; i < NUM_AREA_TYPE ; i++ ){
		property.dec_area[ i ].p1.x = atof( getWord( fp ) );
		property.dec_area[ i ].p1.y = atof( getWord( fp ) );
		property.dec_area[ i ].p2.x = atof( getWord( fp ) );
		property.dec_area[ i ].p2.y = atof( getWord( fp ) );
		property.dec_area[ i ].p3.x = atof( getWord( fp ) );
		property.dec_area[ i ].p3.y = atof( getWord( fp ) );
		property.dec_area[ i ].p4.x = atof( getWord( fp ) );
		property.dec_area[ i ].p4.y = atof( getWord( fp ) );
		property.stop_area[ i ].p1.x = atof( getWord( fp ) );
		property.stop_area[ i ].p1.y = atof( getWord( fp ) );
		property.stop_area[ i ].p2.x = atof( getWord( fp ) );
		property.stop_area[ i ].p2.y = atof( getWord( fp ) );
		property.stop_area[ i ].p3.x = atof( getWord( fp ) );
		property.stop_area[ i ].p3.y = atof( getWord( fp ) );
		property.stop_area[ i ].p4.x = atof( getWord( fp ) );
		property.stop_area[ i ].p4.y = atof( getWord( fp ) );
		property.avoid_area[ i ].p1.x = atof( getWord( fp ) );
		property.avoid_area[ i ].p1.y = atof( getWord( fp ) );
		property.avoid_area[ i ].p2.x = atof( getWord( fp ) );
		property.avoid_area[ i ].p2.y = atof( getWord( fp ) );
		property.avoid_area[ i ].p3.x = atof( getWord( fp ) );
		property.avoid_area[ i ].p3.y = atof( getWord( fp ) );
		property.avoid_area[ i ].p4.x = atof( getWord( fp ) );
		property.avoid_area[ i ].p4.y = atof( getWord( fp ) );
	}
	
	return EXIT_SUCCESS;
}
int commonParam::printProperty( void )
{
	printf( "\n<ROBOT INFO>\n" );
	printf( "%8.3f # WIDTH  [m] OF ROBOT\n", property.robot_info.width );
	printf( "%8.3f # LENGTH [m] OF ROBOT\n", property.robot_info.length );
	printf( "%8.3f # WHEELBASE [m] OF ROBOT\n", property.robot_info.wheelbase );

	printf( "\n<GNSS RECIEVER INFO>\n" );
	printf( "%5d # OFFSET X [mm] OF GNSS RECIEVER\n", property.gnss.offset[ _X ] );
	printf( "%5d # OFFSET Y [mm] OF GNSS RECIEVER\n", property.gnss.offset[ _Y ] ); 
	printf( "%5d # OFFSET Z [mm] OF GNSS RECIEVER\n", property.gnss.offset[ _Z ] ); 
	printf( "%5d # SYSTEM ID\n", property.gnss.coordinate_id ); 

	printf( "\n<INITIAL POSITION>\n" );
	printf( "%15.3f # X [m]\n", property.init_pose.x );
	printf( "%15.3f # Y [m]\n", property.init_pose.y );
	printf( "%15.3f # YAW [rad]\n", property.init_pose.theta );
	
	printf( "\n<FUSION INFO>\n" );
	printf( "%8.3f # ALPFA[ X ]\n", property.fusion.alpfa[ _X ] );
	printf( "%8.3f # ALPFA[ Y ]\n", property.fusion.alpfa[ _Y ] );
	printf( "%8.3f # ALPFA[ Z ]\n", property.fusion.alpfa[ _YAW ] );
	
	printf( "\n<NAVIGATION INFO>\n" );
	printf( "%8.3f # MAX. FORWARD VELOCITY [m/s] OF A MOBILE ROBOT\n", property.navi.fvel );
	printf( "%8.3f # MAX. BACKWARD VELOCITY [m/s] OF A MOBILE ROBOT\n", property.navi.bvel );
	printf( "%8.3f # MAX. ANGLUAR VELOCITY [rad/s] OF A MOBILE ROBOT\n", property.navi.ang_vel ); 
	printf( "%8.3f # MAX. ACCELARATION [m/s^2] OF A MOBILE ROBOT\n", property.navi.acc );
	printf( "%8.3f # MAX. ANGLUAR ACCELARATION [rad/s^2] OF A MOBILE ROBOT\n", property.navi.ang_acc ); 
	
	printf( "\n<WP INFO>\n" );
	printf( "%8.3f # TURNING RADIUS [m] OF A MOBILE ROBOT\n", property.wp_info.TR );
	printf( "%11s # PATH FOR WP FILE\n", property.wp_info.path );
	printf( "%40s # FILE NAME FOR WP INFO\n", property.wp_info.filename );

	printf( "\n<CONTROL INFO>\n" );
	printf( "%8.3f # FUSION RATIO FOR HANDLE ANGLE OFFSET\n", property.cntl.beta );
	printf( "%8.3f # ANGLE RATIO BETWEEN HANDLE AND STEERING\n", property.cntl.s_ratio );
	for( int i = 0 ; i < NUM_GAIN_TYPE ; i++ ){
		printf( "%8.3f # GAIN K1 [deg/s] FOR LINE FOLLOW CONTROL\n", property.cntl.l_K1[ i ] );
	}
	for( int i = 0 ; i < NUM_GAIN_TYPE ; i++ ){
		printf( "%8.3f # GAIN K2 [1/s] FOR LINE FOLLOW CONTROL\n", property.cntl.l_K2[ i ] );
	}
	printf( "%8.3f # CLIP [deg] FOR TARGET ANGLE\n", property.cntl.clip_tgtAngle );
	printf( "%8.3f # CLIP [deg] FOR TARGET STEERING ANGLE\n", property.cntl.clip_tgtSteering );
	printf( "%8.3f # MINIMUN STROKE [mm] FOR ACCEL\n", property.cntl.minStroke );
	printf( "%8.3f # MAXIMUN STROKE [mm] FOR ACCEL\n", property.cntl.maxStroke );
	printf( "%8.3f # GAIN K1 [mm/(m/s)] FOR VELOCITY CONTROL\n", property.cntl.v_K1 );
	printf( "%8.3f # GAIN K1 [mm/ratio] FOR VELOCITY CONTROL\n", property.cntl.v_K2 );
	printf( "%8d # TIME [s] UNTIL THE CUTTING UNIT IS SET\n", property.cntl.cuttingUnit_Down_time );
	printf( "%8d # TIME [s] UNTIL THE CUTTING UNIT IS RETURNED\n", property.cntl.cuttingUnit_Up_time );

	printf( "\n<MOTOR INFO>\n" );
	printf( "%8.3f # THRESHOLD TORQUE [ratio] FOR HANDLE MOTER\n", property.motor.thrHandleTorque );
	printf( "%8.3f # INVERSE INPUT [deg] WHEN BEYOND TORQUE THRESHOLD\n", property.motor.invHandleInput );
	printf( "%8.3f # RETURNED ANGLE [deg] WHEN BEYOND TORQUE THRESHOLD\n", property.motor.diffHandleAngle );
	printf( "%8.3f # MAXIMUN ANGLUAR VELOCITY [deg/s] FOR HANDLE MOTER\n", property.motor.h_angvel );
	printf( "%8.3f # MAXIMUN ANGLUAR ACCELARATION [deg/s^2] FOR HANDLE MOTER\n", property.motor.h_angacc );
	printf( "%8.3f # MAXIMUN VELOCITY [mm/s] FOR ACCEL MOTER\n", property.motor.a_vel );
	printf( "%8.3f # MAXIMUN ACCELARATION [mm/s^2] FOR ACCEL MOTER\n", property.motor.a_acc );
	
	if( flag_all_print ){
		printf( "\n<3D URG INFO>\n" );
		printf( "%5d # ROLL [degree] OF 3D URG\n", property.urg3d.rot[ _ROLL ] );
		printf( "%5d # PITCH [degree] OF 3D URG\n", property.urg3d.rot[ _PITCH ] );
		printf( "%5d # YAW [degree] OF 3D URG\n", property.urg3d.rot[ _YAW ] );
		printf( "%5d # OFFSET X [mm] OF 3D URG\n", property.urg3d.offset[ _X ] );
		printf( "%5d # OFFSET Y [mm] OF 3D URG\n", property.urg3d.offset[ _Y ] ); 
		printf( "%5d # OFFSET Z [mm] OF 3D URG\n", property.urg3d.offset[ _Z ] ); 
	
		printf( "\n<OBSTACLE DETECTION INFO>\n" );
		for( int i = 0 ; i < NUM_AREA_TYPE ; i++ ){
			printf( "<TYPE #%d>\n", i );
			printf( "%8.3f # P1 X [m] FOR DECELERATION AREA\n", property.dec_area[ i ].p1.x );
			printf( "%8.3f #    Y [m] \n", property.dec_area[ i ].p1.y );
			printf( "%8.3f # P2 X [m] FOR DECELERATION AREA\n", property.dec_area[ i ].p2.x );
			printf( "%8.3f #    Y [m] \n", property.dec_area[ i ].p2.y );
			printf( "%8.3f # P3 X [m] FOR DECELERATION AREA\n", property.dec_area[ i ].p3.x );
			printf( "%8.3f #    Y [m] \n", property.dec_area[ i ].p3.y );
			printf( "%8.3f # P4 X [m] FOR DECELERATION AREA\n", property.dec_area[ i ].p4.x );
			printf( "%8.3f #    Y [m] \n\n", property.dec_area[ i ].p4.y );
			printf( "%8.3f # P1 X [m] FOR EMERGENCY STOP AREA\n", property.stop_area[ i ].p1.x );
			printf( "%8.3f #    Y [m] \n", property.stop_area[ i ].p1.y );
			printf( "%8.3f # P2 X [m] FOR EMERGENCY STOP AREA\n", property.stop_area[ i ].p2.x );
			printf( "%8.3f #    Y [m] \n", property.stop_area[ i ].p2.y );
			printf( "%8.3f # P3 X [m] FOR EMERGENCY STOP AREA\n", property.stop_area[ i ].p3.x );
			printf( "%8.3f #    Y [m] \n", property.stop_area[ i ].p3.y );
			printf( "%8.3f # P4 X [m] FOR EMERGENCY STOP AREA\n", property.stop_area[ i ].p4.x );
			printf( "%8.3f #    Y [m] \n\n", property.stop_area[ i ].p4.y );
			printf( "%8.3f # P1 X [m] FOR AVOIDANCE AREA\n", property.avoid_area[ i ].p1.x );
			printf( "%8.3f #    Y [m] \n", property.avoid_area[ i ].p1.y );
			printf( "%8.3f # P2 X [m] FOR AVOIDANCE AREA\n", property.avoid_area[ i ].p2.x );
			printf( "%8.3f #    Y [m] \n", property.avoid_area[ i ].p2.y );
			printf( "%8.3f # P3 X [m] FOR AVOIDANCE AREA\n", property.avoid_area[ i ].p3.x );
			printf( "%8.3f #    Y [m] \n", property.avoid_area[ i ].p3.y );
			printf( "%8.3f # P4 X [m] FOR AVOIDANCE AREA\n", property.avoid_area[ i ].p4.x );
			printf( "%8.3f #    Y [m] \n\n", property.avoid_area[ i ].p4.y );
		}
	}
	
	return EXIT_SUCCESS;
}
int commonParam::saveProperty( void )
{
	fprintf( fp_config, "# ****** for ROBOT INFO *************************\n" );
	fprintf( fp_config, "# WIDTH [m]		LENGTH [m]\n" );
	fprintf( fp_config, "	%f		%f\n", property.robot_info.width, property.robot_info.length );
	fprintf( fp_config, "# WHEELBASE [m]\n" );
	fprintf( fp_config, "	%f\n", property.robot_info.wheelbase );
	fprintf( fp_config, "\n" );
	
	fprintf( fp_config, "# ****** for GNSS Antenna ***************************\n" );
	fprintf( fp_config, "# offset_x[mm]		offset_y[mm]		offset_z[mm]\n" );
	fprintf( fp_config, "	%d		%d		%d\n", property.gnss.offset[ _X ], property.gnss.offset[ _Y ], property.gnss.offset[ _Z ] );
	fprintf( fp_config, "# Zero-Point numner # 1系 or 2系.....\n" );
	fprintf( fp_config, "	%d\n", property.gnss.coordinate_id );
	fprintf( fp_config, "\n" );

	fprintf( fp_config, "# ****** for INITIAL POSITION *************************\n" );
	fprintf( fp_config, "# x [m]				y [m]			theta [rad]\n" );
	fprintf( fp_config, "%f		%f		%f\n", property.init_pose.x, property.init_pose.y, property.init_pose.theta );
	fprintf( fp_config, "\n" );

	fprintf( fp_config, "# ****** for FUSION INFO *************************\n" );
	fprintf( fp_config, "# alpha[z]のみ使用中\n" );
	fprintf( fp_config, "# alpfa[x]			alpfa[y]			alpfa[z]\n" );
	fprintf( fp_config, "	%f			%f				%f\n", property.fusion.alpfa[ _X ], property.fusion.alpfa[ _Y ], property.fusion.alpfa[ _YAW ] );
	fprintf( fp_config, "\n" );

	fprintf( fp_config, "# ****** for NAVIGATION *************************\n" );
	fprintf( fp_config, "# velocityのみ使用中\n" );
	fprintf( fp_config, "# Forward Vel. [m/s]	Backward Vel. [m/s]		Angluar vel. [rad/s]	Acc. [m/s^2]	Angluar acc. [rad/\n" );
	fprintf( fp_config, "	%f				%f				%f					%f				%f\n", property.navi.fvel, property.navi.bvel, property.navi.ang_vel, property.navi.acc, property.navi.ang_acc );
	fprintf( fp_config, "\n" );
	
	fprintf( fp_config, "# ****** for WP INFO ****************************\n" );
	fprintf( fp_config, "# Turning Radius [m]\n" );
	fprintf( fp_config, "	%f\n", property.wp_info.TR );
	fprintf( fp_config, "# WP file\n" );
	fprintf( fp_config, "%s\n", property.wp_info.filename );	// 要修正
//	fprintf( fp_config, "%s\n", WPfile );
	fprintf( fp_config, "\n" );
	
	fprintf( fp_config, "# ****** for CONTROL INFO ****************************\n" );
	fprintf( fp_config, "#	double beta;	// ハンドル角オフセットの推定割合\n" );
	fprintf( fp_config, "	%f\n", property.cntl.beta );
	fprintf( fp_config, "#	double s_ratio;;	//(ハンドル角/ステアリング角)の比\n" );
	fprintf( fp_config, "	%f\n", property.cntl.s_ratio );
	fprintf( fp_config, "#// ハンドルの制御用ゲインの設定\n" );
	fprintf( fp_config, "#	double l_K1;	// ライン追従制御用ゲイン//[deg/m]\n" );
	for( int i = 0 ; i < NUM_GAIN_TYPE ; i++ ){
		fprintf( fp_config, "	%f ", property.cntl.l_K1[ i ] );
	}
	fprintf( fp_config, "\n" );
	fprintf( fp_config, "#	double l_K2;	// ライン追従制御用ゲイン//[1/s]\n" );
	for( int i = 0 ; i < NUM_GAIN_TYPE ; i++ ){
		fprintf( fp_config, "	%f ", property.cntl.l_K2[ i ] );
	}
	fprintf( fp_config, "\n" );
	fprintf( fp_config, "#	double clip_tgtAngle;		// 目標侵入角度のクリップ角 [deg]\n" );
	fprintf( fp_config, "	%f\n", property.cntl.clip_tgtAngle );
	fprintf( fp_config, "#	double clip_tgtSteering;	// 目標ステアリング角のクリップ角 [deg]\n" );
	fprintf( fp_config, "	%f\n", property.cntl.clip_tgtSteering );
	fprintf( fp_config, "#// アクセルの制御用ゲインの設定\n" );
	fprintf( fp_config, "#	double minStroke;	// [mm]\n" );
	fprintf( fp_config, "	%f\n", property.cntl.minStroke );
	fprintf( fp_config, "#	double maxStroke;	// [mm]\n" );
	fprintf( fp_config, "	%f\n", property.cntl.maxStroke );
	fprintf( fp_config, "#	double v_K1;		// 速度制御用微分ゲイン	// 1[m/s]の差に対して何mm[ex.5mm]のストローク変化\n" );
	fprintf( fp_config, "	%f\n", property.cntl.v_K1 );
	fprintf( fp_config, "#	double v_K2;		// 速度制御用積分ゲイン	// 変化率１に対して、何mm[ex.0.5mm]のストローク\n" );
	fprintf( fp_config, "	%f\n", property.cntl.v_K2 );
	fprintf( fp_config, "#// カッティングユニット用レバーのニュートラルポジションへ戻すまでの時間" );
	fprintf( fp_config, "#	unsigned int cuttingUnit_Down_time;	// 芝刈部を降ろす時間" );
	fprintf( fp_config, "#	unsigned int cuttingUnit_Up_time;	// 芝刈部を上げる時間" );
	fprintf( fp_config, "	%d ", property.cntl.cuttingUnit_Down_time );
	fprintf( fp_config, "	%d\n", property.cntl.cuttingUnit_Up_time );
	fprintf( fp_config, "\n" );

	fprintf( fp_config, "# ****** for MOTER INFO ****************************\n" );
	fprintf( fp_config, "#// ハンドルを振切った際の設定\n" );
	fprintf( fp_config, "#	double maxHandleTorque;	// ハンドルの最大トルク [０〜１０００]\n" );
	fprintf( fp_config, "	%f\n", property.motor.thrHandleTorque );
	fprintf( fp_config, "#	double invHandleInput;	// 振切った際の逆回転入力（絶対値） [deg]\n" );
	fprintf( fp_config, "	%f\n", property.motor.invHandleInput );
	fprintf( fp_config, "#	double diffHandleAngle;	// 振切った際にハンドル角を戻す相対角度 [deg]\n" );
	fprintf( fp_config, "	%f\n", property.motor.diffHandleAngle );
	fprintf( fp_config, "#// 中空アクチュエータの設定（ハンドル）\n" );
	fprintf( fp_config, "#	double h_angvel;	// [deg/s]\n" );
	fprintf( fp_config, "	%f\n", property.motor.h_angvel );
	fprintf( fp_config, "#	double h_angacc;	// [deg/s^2]\n" );
	fprintf( fp_config, "	%f\n", property.motor.h_angacc );
	fprintf( fp_config, "#// リニアアクチュエータの設定（アクセル）\n" );
	fprintf( fp_config, "#	double a_vel;		// [mm/s]	//200 or 600\n" );
	fprintf( fp_config, "	%f\n", property.motor.a_vel );
	fprintf( fp_config, "#	double a_acc;		// [mm/s^2]\n" );
	fprintf( fp_config, "	%f\n", property.motor.a_acc );
	fprintf( fp_config, "\n" );
	
	fprintf( fp_config, "# ****** for 3D URG ********************************\n" );
	fprintf( fp_config, "# Roll		Pitch	Yaw [degree]\n" );
	fprintf( fp_config, "	%d		%d		%d\n", property.urg3d.rot[ _ROLL ], property.urg3d.rot[ _PITCH ], property.urg3d.rot[ _YAW ] );
	fprintf( fp_config, "# offset_x[mm]		offset_y[mm]		offset_z[mm]\n" );
	fprintf( fp_config, "	%d					%d					%d\n", property.urg3d.offset[ _X ], property.urg3d.offset[ _Y ], property.urg3d.offset[ _Z ] );
	fprintf( fp_config, "\n" );
	
	fprintf( fp_config, "# ****** for OBSTACLE DETECTION *****************\n" );
	fprintf( fp_config, "# P1 : Left Bottom, P2 : Right Bottom\n" );
	fprintf( fp_config, "# P3 : Right Top,   P4 : Left Top\n" );
	for( int i = 0 ; i < NUM_AREA_TYPE ; i++ ){
		fprintf( fp_config, "# < TYPE #%d >\n", i );
		fprintf( fp_config, "# Deceleration area\n" );
		fprintf( fp_config, "# P1 x [m]		P1 y [m]		P2 x [m]		P2 y [m]\n" );
		fprintf( fp_config, "	%f				%f			%f				%f\n", 
					property.dec_area[ i ].p1.x, property.dec_area[ i ].p1.y, property.dec_area[ i ].p2.x, property.dec_area[ i ].p2.y );
		fprintf( fp_config, "# P3 x [m]		P3 y [m]		P4 x [m]		P4 y [m]\n" );
		fprintf( fp_config, "	%f				%f				%f				%f\n", 
					property.dec_area[ i ].p3.x, property.dec_area[ i ].p3.y, property.dec_area[ i ].p4.x, property.dec_area[ i ].p4.y );
		fprintf( fp_config, "# Emergency stop \n" );
		fprintf( fp_config, "# P1 x [m]		P1 y [m]		P2 x [m]		P2 y [m]\n" );
		fprintf( fp_config, "	%f				%f			%f				%f\n",
					property.stop_area[ i ].p1.x, property.stop_area[ i ].p1.y, property.stop_area[ i ].p2.x, property.stop_area[ i ].p2.y );
		fprintf( fp_config, "# P3 x [m]		P3 y [m]		P4 x [m]		P4 y [m]\n" );
		fprintf( fp_config, "	%f				%f				%f				%f\n", 
					property.stop_area[ i ].p3.x, property.stop_area[ i ].p3.y, property.stop_area[ i ].p4.x, property.stop_area[ i ].p4.y );
		fprintf( fp_config, "# Avoidance \n" );
		fprintf( fp_config, "# P1 x [m]		P1 y [m]		P2 x [m]		P2 y [m]\n" );
		fprintf( fp_config, "	%f				%f			%f				%f\n", 
					property.avoid_area[ i ].p1.x, property.avoid_area[ i ].p1.y, property.avoid_area[ i ].p2.x, property.avoid_area[ i ].p2.y );
		fprintf( fp_config, "# P3 x [m]		P3 y [m]		P4 x [m]		P4 y [m]\n" );
		fprintf( fp_config, "	%f				%f				%f				%f\n", 
					property.avoid_area[ i ].p3.x, property.avoid_area[ i ].p3.y, property.avoid_area[ i ].p4.x, property.avoid_area[ i ].p4.y );
	}
	
	return EXIT_SUCCESS;
}
