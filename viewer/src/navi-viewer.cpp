/*
 * navi-viewer
 * 
 * Create : 2018.02.04
 * Update : 2019.09.05
 * Author : T.Hasegawa
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>

//#include "gnss.hpp"
#include "localizer.hpp"
#include "config.hpp"
#include "wp.hpp"
#include "GraphDrawerTrajectory.hpp"
#include "utility.hpp"
#include "control.hpp"
#include "framework.hpp"

#define RANGE 10	// 描画範囲[m]

static int gShutOff = 0;
static void setSigInt(  );
static int printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 100;
static int keyframeSkip = 10;		// キーフレーム間隔
static bool flag_save_control = false;
static bool flag_save_localizer = false;
//static bool flag_monitor_position = false;

static SSMApi <config, config_property> *CONF;
static SSMApi <localizer> *LOCAL;
static SSMApi <wp_gl> *WP;
static SSMApi <control> *CNTL;
int main( int aArgc, char *aArgv[ ] )
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
		
	try {
		setupSSM( );
		setSigInt( );
				
		NAVI_VIEWER_CLASS tdrawer;			// gnuplotによる描画
		tdrawer.setParameters( &config.property, flag_save_localizer, flag_save_control );
		tdrawer.initGnuplot( );				// gnuplot初期化
		tdrawer.setGrid( );
		tdrawer.setAspectRatio( -0.9 );		// x軸とy軸の比（負にすると中身が一定）
	
		unsigned long counter = 0;
//		double start_time = gettimeSSM( );
			
		bool update[ 3 ] = { false };
		SSM_tid update_id[ 3 ] = { -1 };
#define INDEX_LOCALIZER		0
#define INDEX_WP			1
#define INDEX_CONTROL		2

		//if( flag_monitor_position ){
			//tdrawer.setWP( );
		//} else {
			// 初期値を入力する
			SSM_tid tid = getTID_top( wp_gl.getSSMId( ) );
			if( tid > 0 ){
				wp_gl.read( tid-1 );	// MUST check
//				wp_gl.read( tid );	// MUST check
				tdrawer.setWP( &wp_gl.data );
			}
			wp_gl.readLast(  );
			tdrawer.setWP( &wp_gl.data );
//		}

		Gprint( "Start navi-viewer\n" );
		while( !gShutOff ){
			update[ INDEX_LOCALIZER ] = update[ INDEX_WP ] = update[ INDEX_CONTROL ] = false;

			// get latest data for INDEX_LOCALIZER
			if( update_id[ INDEX_LOCALIZER ] < getTID_top( localizer.getSSMId( ) ) ){
				localizer.readLast(  );
				update[ INDEX_LOCALIZER ] = true; // 最新情報を読み込む
				update_id[ INDEX_LOCALIZER ] = localizer.timeId;
			} else {
				update[ INDEX_LOCALIZER ] = false;
			}
			// get latest data for INDEX_WP
//			if( ( update_id[ INDEX_WP ] < getTID_top( wp_gl.getSSMId( ) ) ) && !flag_monitor_position ){
			if( update_id[ INDEX_WP ] < getTID_top( wp_gl.getSSMId( ) ) ){
				wp_gl.readLast(  );
				update[ INDEX_WP ] = true; // 最新情報を読み込む
				update_id[ INDEX_WP ] = wp_gl.timeId;
			} else {
				update[ INDEX_WP ] = false;
			}
			// get latest data for INDEX_CONTROL
			if( ( update_id[ INDEX_CONTROL ] < getTID_top( control.getSSMId( ) ) ) && flag_save_control ){
				control.readLast(  );
				update[ INDEX_CONTROL ] = true; // 最新情報を読み込む
				update_id[ INDEX_CONTROL ] = control.timeId;
			} else {
				update[ INDEX_CONTROL ] = false;
			}
						
//			if( update[ INDEX_WP ] && !flag_monitor_position ){
			if( update[ INDEX_WP ] ){
				tdrawer.setWP( &wp_gl.data );
			}
			
			if( update[ INDEX_CONTROL ] ){
				if( flag_save_control ){
					tdrawer.setControlInfo( &control.data, control.time );
				}
			}
			
			if( update[ INDEX_LOCALIZER ] ){
				tdrawer.setPose( &localizer.data, localizer.time );
							
				//tdrawer.setPose( &localizer.data );
				//if( flag_save ){
					//tdrawer.log2txt_localizer( &localizer.data, localizer.time, start_time );
				//}
				if( counter >= keyframeSkip ){			// キーフレームのときだけ行う
					// 描画範囲
					tdrawer.setRange( localizer.data.estPose.x-RANGE, localizer.data.estPose.x+RANGE, localizer.data.estPose.y-RANGE, localizer.data.estPose.y+RANGE );
					tdrawer.drawGraph( );
					counter = 0;
				}
				counter++;
				
			} else {
				usleepSSM( dT * 1000 );
			}
		}
//		if( flag_save ) tdrawer.closeSaveFile( );

	}
	catch (std::runtime_error const & error){
		std::cout << error.what() << std::endl;
	}
	catch (...){
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}

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
	fprintf( stderr, "\t$ %s [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -t 100 -s\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-l | --localizer SAVE   : Save localizer data to log_localizer.dat (default=%d)\n", flag_save_localizer );
	printf( "\t-c | --control   SAVE   : Save control data to log_control.dat (default=%d)\n", flag_save_control );
//	printf( "\t-m | --monitor   MONITOR: Monitor postion of vichle (defautl=%d)\n", flag_monitor_position );
	printf( "\t-t | --s_time    TIME   : Wait time (defautl=%dms)\n", dT );
	printf( "\n" );
	return EXIT_SUCCESS;
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "s_time", 1, 0, 't'},
		{ "localizer", 1, 0, 'l' },
		{ "control", 1, 0, 'c' },
//		{ "monitor", 1, 0, 'm' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "t:lch", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 't':
			dT = atoi( optarg );
			break;
		case 'l':
			flag_save_localizer = true;
			break;
		case 'c':
			flag_save_control = true;
			break;
		//case 'm':
			//flag_monitor_position = true;
			//flag_save_control = false;
			//break;
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
//	if( !flag_monitor_position ){
		std::cerr << "open  wp_gl ... ";
		if( !WP->open( SSM_READ ) )
			throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open wp_gl on ssm.\n" );
		else
			std::cerr << "OK.\n";
//	}
	
	// controlを開く
	if( flag_save_control ){
		std::cerr << "open  control ... ";
		if( !CNTL->open( SSM_READ ) )
			throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open control on ssm.\n" );
		else
			std::cerr << "OK.\n";	
	}
	
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
	if( flag_save_control ) CNTL->release( );
	endSSM( );
	Gprint( "end\n" );
}

