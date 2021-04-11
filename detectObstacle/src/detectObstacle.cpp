/*
 * Author : T.Hasegawa
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <ypspur.h>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include <ssmtype/spur-odometry.h>
#include "detectObstacle.hpp"
#include "obp.hpp"
#include "config.hpp"
#include "utility.hpp"
#include "wp.hpp"
#include "urg.hpp"
#include "framework.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 100;
static int sensor_id = 0;

static SSMApi <obp_fs> *OBPoint;
static SSMApi <urg_fs, urg_property> *URG;;
static SSMApi <config, config_property> *CONF;
static SSMApi <wp_gl> *WP;

int main( int aArgc, char *aArgv[] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi <obp_fs> obp_fs( OBP_SNAME, 0 );
		OBPoint = &obp_fs;
	SSMApi <urg_fs, urg_property> urg_fs( URG_SNAME, sensor_id );
		URG = &urg_fs;
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;
	SSMApi <wp_gl> wp_gl( WP_SNAME, 0 );
		WP = &wp_gl;

	try {
		setupSSM( );
		setSigInt( );

		detectObstacle_CLASS obp;
		obp.setParameter( config.property );

		bool update[ 2 ] = { false, false };
		SSM_tid update_id[ 2 ] = { -1 };
#define INDEX_URG	0
#define INDEX_WP	1

		Gprint( "\nStart detectObstacle\n" );
		while( !gShutOff ){
			update[ INDEX_URG ] = update[ INDEX_WP ] = false;
			// get latest data for INDEX_URG
			if( update_id[ INDEX_URG ] < getTID_top( urg_fs.getSSMId( ) ) ){
				urg_fs.readLast(  );
				update[ INDEX_URG ] = true; // 最新情報を読み込む
				update_id[ INDEX_URG ] = urg_fs.timeId;
			} else {
				update[ INDEX_URG ] = false;
			}
			// get latest data for INDEX_WP
			if( update_id[ INDEX_WP ] < getTID_top( wp_gl.getSSMId( ) ) ){
				wp_gl.readLast(  );
				update[ INDEX_WP ] = true; // 最新情報を読み込む
				update_id[ INDEX_WP ] = wp_gl.timeId;
			} else {
				update[ INDEX_WP ] = false;
			}

			if( update[ INDEX_WP ] ){
				obp.setAreaType( wp_gl.data.area_type );
			}
			if( update[ INDEX_URG ] ){
				obp.chkObstacle( &urg_fs.data );
				obp_fs.data = obp.getObpData( );
				obp_fs.write( urg_fs.time );
				
//				obp.printObstData( );	// 確認用
					
			} else {
				usleepSSM( dT * 1000 );
			}
		}
	}
	catch( std::runtime_error const & error ){
		std::cout << error.what( ) << std::endl;
	}
	catch( ... ){
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
	fprintf( stderr, "\t$ %s DEVICE_PATH [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -t 100\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-t | --s_time       TIME   : Sampling time (defautl=%dms)\n", dT );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "s_time", 1, 0, 't'},
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "t:h", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
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
	if( !initSSM() )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to initialize ssm.");
	else
		std::cerr << "OK.\n";

	// urgを開く
	std::cerr << "open urg_fs ... ";
	if( !URG->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open urg on ssm.\n" );
	else
		std::cerr << "OK.\n";

	// obp_fsを開く
	std::cerr << "open obp_fs ... ";
	if( !OBPoint->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open obp_fs on ssm.\n" );
	else
		std::cerr << "OK.\n";	
	
	// wp_glを開く
	std::cerr << "open wp_gl ... ";
	if( !WP->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open wp_gl on ssm.\n" );
	else
		std::cerr << "OK.\n";
		
	// configからデータを取得
	std::cerr << "get onfig.property ... ";
	if( !CONF->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to get config.property.");
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	CONF->release( );
	URG->release( );
	OBPoint->release( );
	WP->release( );
	endSSM( );
	Gprint( "end\n" );
}
