/*
 * Navigation and Line-trace control for a mobile robot.
 * 
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

#include <ypspur.h>
#include <ssm.hpp>
#include <joystick.hpp>
#include "system-mgr.hpp"
#include "config.hpp"
#include "utility.hpp"
#include "localizer.hpp"
#include "control.hpp"
#include "OMcntl.hpp"

static int gShutOff = 0;
static void setSigInt(  );
static int printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 10;
static unsigned int wp_id = 1;
static char device_name[ 128 ] = "/dev/ttyUSB0";
static bool flag_analysis = false;
static bool flag_joystick = true;
static bool flag_positioning_process = false;

static SSMApi <config, config_property> *CONF;
static SSMApi <localizer> *LOCAL;
static SSMApi <wp_gl> *WP;
static SSMApi <control> *CNTL;
static SSMApi<ssm::JSData, ssm::JSProperty> *JOY;
static SSMApi<OMcntl> *OMotor;
static SSMApi <obp_fs> *OBPoint;

int main( int aArgc, char *aArgv[] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;
	SSMApi<localizer> localizer( LOCALIZER_SNAME, 0 );
		LOCAL = &localizer;
	SSMApi <wp_gl> wp_gl( WP_SNAME, 0 );
		WP = &wp_gl;
	SSMApi <control> control( CONTROL_SNAME, 0 );
		CNTL = &control;
	SSMApi<ssm::JSData, ssm::JSProperty> joystick( SSM_NAME_JOYSTICK, 0 );
		JOY = &joystick;
	SSMApi<OMcntl> motor( OM_CNTL_SNAME, 0 );
		OMotor = &motor;
	SSMApi<obp_fs> obp_fs( OBP_SNAME, 0 );
		OBPoint = &obp_fs;
		
	SystemMgr_CLASS navi;
		
	try {
		setupSSM( );
		setSigInt( );
		
		if( flag_positioning_process ){
			navi.initilize( config.property, device_name, flag_analysis, wp_id, POSITIONING );
		} else {
			navi.initilize( config.property, device_name, flag_analysis, wp_id, NAVI );
		}
		
		Gprint( "Start Navigator\n" );
		// スタート位置を書き込み
		wp_gl.data = navi.getWP( );
		wp_gl.write( );
		navi.CountUpWP( );
		
		bool update[ 3 ] = { false };
		SSM_tid update_id[ 3 ] = { -1 };
#define INDEX_LOCALIZER		0
#define INDEX_JOYSTICK		1
#define INDEX_OBP			2

		while( !gShutOff ){
			
			// Write the WP information to SSM.
			localizer.readLast( );
			wp_gl.data = navi.getWP( &localizer.data );
			wp_gl.write( );
			
			navi.setCuttingUnit( );

			while( !navi.chkUpdateWP( ) ){
				
				update[ INDEX_LOCALIZER ] = update[ INDEX_JOYSTICK ] = update[ INDEX_OBP ] = false;
				// get latest data for INDEX_LOCALIZER
				if( update_id[ INDEX_LOCALIZER ] < getTID_top( localizer.getSSMId( ) ) ){
					localizer.readLast(  );
					update[ INDEX_LOCALIZER ] = true; // 最新情報を読み込む
					update_id[ INDEX_LOCALIZER ] = localizer.timeId;
				} else {
					update[ INDEX_LOCALIZER ] = false;
				}
				// get latest data for INDEX_JOYSTICK
				if( ( update_id[ INDEX_JOYSTICK ] < getTID_top( joystick.getSSMId( ) ) ) && flag_joystick ){
					joystick.readLast(  );
					update[ INDEX_JOYSTICK ] = true; // 最新情報を読み込む
					update_id[ INDEX_JOYSTICK ] = joystick.timeId;
				} else {
					update[ INDEX_JOYSTICK ] = false;
				}
				// get latest data for INDEX_OBP
				if( update_id[ INDEX_OBP ] < getTID_top( obp_fs.getSSMId( ) ) ){
					obp_fs.readLast(  );
					update[ INDEX_OBP ] = true; // 最新情報を読み込む
					update_id[ INDEX_OBP ] = obp_fs.timeId;
				} else {
					update[ INDEX_OBP ] = false;
				}
				
				// 緊急停止の処理
				if( update[ INDEX_JOYSTICK ] && flag_joystick ){
					navi.chkEmergency4Joystick( &joystick.data );
				}
				// 衝突回避の処理（停止のみ）
				if( update[ INDEX_OBP ] ){
					navi.collisionAvoidance( &obp_fs.data );
				}
				
				// 制御系
				if( update[ INDEX_LOCALIZER ] ){
					navi.ControlVelocity( &localizer.data, localizer.time );		// 速度制御
					navi.followLine( &localizer.data, localizer.time );	// ライン走行制御
					// 芝刈部の戻し処理
					navi.returnCuttingUnit( );
				}
				
				// モーターへの出力
				bool ret = navi.sendHandleOrder( );
				if( ret ){
					control.data = navi.getControlData( );
					control.write( );
					if( !flag_analysis ){
						motor.data = navi.getMotorData( );
						motor.write( );
					}
				}

			}
			
			if( navi.chkGoal( ) ) break;

		}
		if( !flag_analysis ){
			navi.stopVehicle(  );
			usleepSSM( 1.0 * 1000 * 1000 );
		}
	}
	
	catch ( std::runtime_error const & error ){
		std::cout << error.what() << std::endl;
	}
	catch (...){
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}
	
	navi.closeAllProcess( );
	Terminate( );

	return EXIT_SUCCESS;
}
static void ctrlC( int aStatus )
{
	signal( SIGINT, NULL );
	gShutOff = 1;
}
static void setSigInt(  )
{
	struct sigaction sig;
	memset( &sig, 0, sizeof ( sig ) );
	sig.sa_handler = ctrlC;
	sigaction( SIGINT, &sig, NULL );
}
static int printShortHelp( const char *programName )
{
	fputs( "HELP\n", stderr );
	fprintf( stderr, "\t$ %s DEVICE_PATH [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -t 100 -w 1\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-w | --wp_id   	WP ID    : Numbe of WP for start position (default=%d)\n", wp_id );
	printf( "\t-a | --analysis           : analysis mode (defautl=%d)\n", flag_analysis );
	printf( "\t-d | --device    DEVICE   : set device port for motor driver (default=%s)\n", device_name );
	printf( "\t-j | --joystick  JOYSTICK : Not use joystick for emergency (default=%d)\n", flag_joystick );
	printf( "\t-p | --positioning        : Use the positioning process (default=%d)\n", flag_positioning_process );
	printf( "\t-t | --s_time    TIME     : Wait time (defautl=%dms)\n", dT );
	printf( "\n" );
	return EXIT_SUCCESS;
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "wp_id", 1, 0, 'w' },
		{ "analysis", 1, 0, 'a'},
		{ "device", 1, 0, 'd' },
		{ "joystick", 1, 0, 'j' },
		{ "positioning", 1, 0, 'p' },
		{ "s_time", 1, 0, 't'},
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "w:d:t:ajlph", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'w':
			wp_id = atoi( optarg );
			if( wp_id <= 0 ){
				fprintf( stderr, "ERROR; wp_id >= 1 \n" );
				exit( EXIT_FAILURE );
			}
			break;
		case 'd':
			sprintf( device_name, "%s", optarg );
			break;
		case 'a':
			flag_analysis = true;
			flag_joystick = false;
			break;
		case 'j':
			flag_joystick = false;
			break;
		case 'p':
			flag_positioning_process = false;
			break;
		case 't':
			dT = atoi( optarg );
			break;
		case 'h':
			printShortHelp( aArgv[0] );
			return false;
			break;
		default:
			fprintf( stderr, "help : %s -h\n", aArgv[0] );
			return false;
			break;
		}
	}
	return true;
}
static void setupSSM( void )
{
	std::cerr << "initializing ssm ... ";
	if( !initSSM( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to initialize ssm.");
	else
		std::cerr << "OK.\n";
		
	// localizerを開く
	std::cerr << "open  localizer... ";
	if( !LOCAL->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open localizer on ssm.\n" );
	else
		std::cerr << "OK.\n";

	// wp_glを開く
	std::cerr << "open  wp_gl... ";
	if( !WP->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open wp_gl on ssm.\n" );
	else
		std::cerr << "OK.\n";
		
	// controlを生成
	std::cerr << "create control ... ";
	if( !CNTL->create( 5, ( double )dT/1000.0 ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create control on ssm.\n" );
	else
		std::cerr << "OK.\n";
		
	// joystick を開く
	if( flag_joystick ){
		std::cerr << "open joystick ... ";
		if( !JOY->open( SSM_READ ) )
			throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open joystick on ssm.\n" );
		else
			std::cerr << "OK.\n";
	}

	// OMcntl を生成
	if( !flag_analysis ){
		std::cerr << "create OMcntl ... ";
		if( !OMotor->create( 5, ( double )dT/1000.0 ) )
			throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create OMcntl on ssm.\n" );
		else
			std::cerr << "OK.\n";
	}
	// obp_fsを開く
	std::cerr << "open obp_fs ... ";
	if( !OBPoint->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open obp_fs on ssm.\n" );
	else
		std::cerr << "OK.\n";
	// get config.property
	std::cerr << "get config.property ... ";
	if( !CONF->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to get config.property.");
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	CONF->release( );
	WP->release( );
	LOCAL->release( );
	CNTL->release( );
	OMotor->release( );
	OBPoint->release( );
	if( flag_joystick ) JOY->release( );
	endSSM( );
	Gprint( "end\n" );
}
