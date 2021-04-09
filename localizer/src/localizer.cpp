#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include <ypspur.h>
#include <ssmtype/spur-odometry.h>
#include "config.hpp"
#include "localizer.hpp"
#include "utility.hpp"
#include "imu.hpp"
#include "estimatePose.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 10;	// 10ms
static bool flag_setInitPose = true;
static unsigned int wp_id = 1;

static SSMApi <config, config_property> *Conf;
static SSMApi <imu_fs> *IMU;
static SSMApi <rtk_gnss_f9p> *GNSS;
static SSMApi <localizer> *LOCAL;

int main( int aArgc, char *aArgv[] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		Conf = &config;
    SSMApi <imu_fs> imu(IMU_SNAME, 0 );
		IMU = &imu;
	SSMApi <rtk_gnss_f9p> rtk_gnss( SNAME_GNSS_F9P, 0 );
		GNSS = &rtk_gnss;
	SSMApi <localizer> localizer( LOCALIZER_SNAME, 0 );
		LOCAL = &localizer;

	try {
		setupSSM( );
		setSigInt( );

		estimatePose estim;
		// 各種変数の設定（初期位置、IMUのオフセット、Alpfaの値）
		estim.setParameters( config.property, flag_setInitPose, wp_id );

		bool update[ 3 ] = { false };
		SSM_tid update_id[ 3 ] = { -1 };
#define INDEX_IMU		0
#define INDEX_GNSS		1

		Gprint( "\nstart localizer\n" );
		while( !gShutOff ){
			update[ INDEX_IMU ] = update[ INDEX_GNSS ] = false;

			// get latest data for INDEX_IMU
			if( update_id[ INDEX_IMU ] < getTID_top( imu.getSSMId( ) ) ){
				imu.readLast(  );
				update[ INDEX_IMU ] = true; // 最新情報を読み込む
				update_id[ INDEX_IMU ] = imu.timeId;
			} else {
				update[ INDEX_IMU ] = false;
			}
			// get latest data for INDEX_GNSS
			if( update_id[ INDEX_GNSS ] < getTID_top( rtk_gnss.getSSMId( ) ) ){
				rtk_gnss.readLast(  );
				update[ INDEX_GNSS ] = true; // 最新情報を読み込む
				update_id[ INDEX_GNSS ] = rtk_gnss.timeId;
			} else {
				update[ INDEX_GNSS ] = false;
			}
	
			// IMUの処理
			if( update[ INDEX_IMU ] ){
				estim.setIMUAngles( &imu.data );
			}
			
			// GNSSの処理
			if( update[ INDEX_GNSS ] ){
				imu.readTime( rtk_gnss.time );
				estim.fusionEstPose( &rtk_gnss.data, &imu.data, &localizer.data );
				localizer.write( rtk_gnss.time );
			}			
		}
		
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
static void printShortHelp( const char *programName )
{
	fputs( "HELP\n", stderr );
	fprintf( stderr, "\t$ %s [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -t 100 -a -i\n", programName );
	printf( "OPTION\n" );
	printf( "\t-t | --s_time      TIME   : Sampling time (defautl=%dms)\n", dT );
	printf( "\t-i | --init_pos           : Use the initial position written in GM_Config.cfg (defautl=%d)(Use:0, NotUse:1)\n", flag_setInitPose );
	printf( "\t-w | --wp_id       WP_ID  : Set WP's id (defautl=%d)\n", wp_id );
//	printf( "\t-g | --gyro_odm    ODM    : Use gyro_odometory (defautl=%d)\n", flag_odm );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "s_time", 1, 0, 't'},
		{ "init_pos", 1, 0, 'i'},
		{ "wp_id", 1, 0, 'w'},
//		{ "gyro_odm", 1, 0, 'g'},
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "t:w:ih", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 't':
			dT = atoi( optarg );    
			break;
		case 'i':
			flag_setInitPose = false;
			break;
		case 'w':
			flag_setInitPose = true;
			wp_id = atoi( optarg );  
			break;
		//case 'g':
			//flag_odm = false;
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
			
	// configのpropertyを取得
	std::cerr << "get config_property ... ";
	if( !Conf->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to config get.");
	else
		std::cerr << "OK.\n";
					
	// imu_fs を開く
	std::cerr << "open imu_fs ... ";
	if( !IMU->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open imu_fs on ssm.\n" );
	else
		std::cerr << "OK.\n";
			
	// rtk_gnss を開く
	std::cerr << "open rtk_gnss ... ";
	if( !GNSS->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open rtk_gnss on ssm.\n" );
	else
		std::cerr << "OK.\n";

	// localizerを開く
	std::cerr << "open localizer ... ";
	if( !LOCAL->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open localizer on ssm.\n" );
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	Conf->release( );
	IMU->release( );
	GNSS->release( );
	LOCAL->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
