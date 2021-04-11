/*
 * detectObstacle-viewer.cpp
 *
 * Date : 2017.03.10
 * Update : 2017.10.01
 *         - Read 2 kinds of area type to detect obstacles
 * Update : 2019.08.22
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include "framework.hpp"
#include "urg.hpp"
#include "detectObstacle.hpp"
#include "config.hpp"
#include "GraphDrawerOBP.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static int sensor_id = 0;
static bool flag_intensity = false;
static bool flag_laser = false;
static unsigned int dT = 100;	// 100ms

static SSMApi <urg_fs> *URG;
static SSMApi <obp_fs> *OBPoint;
static SSMApi <config, config_property> *CONF;

int main( int aArgc, char *aArgv[] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi<urg_fs> urg_fs( URG_SNAME, sensor_id );
		URG = &urg_fs;
	SSMApi<obp_fs> obp_fs( OBP_SNAME, 0 );
		OBPoint = &obp_fs;
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;

	try {
		setupSSM( );
		setSigInt( );
		
		int area_type = 0;
		
		GraphDrawerOBP_CLASS gdrawer;
		gdrawer.setParameter( flag_laser, flag_intensity, &config.property );
		gdrawer.initGnuplot( );				// gnuplot初期化
		gdrawer.setRange( 15, 15 );
		
		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_OBP			0

		Gprint( "\nStart detectObstacle-viewer" );
		while( !gShutOff ){
			update[ INDEX_OBP ] = false;
			// get latest data for INDEX_OBP
			if( update_id[ INDEX_OBP ] < getTID_top( obp_fs.getSSMId( ) ) ){
				obp_fs.readLast(  );
				update[ INDEX_OBP ] = true; // 最新情報を読み込む
				update_id[ INDEX_OBP ] = obp_fs.timeId;
			} else {
				update[ INDEX_OBP ] = false;
			}

			if( update[ INDEX_OBP ] ){
				urg_fs.readTime( obp_fs.time );
				
				gdrawer.setScan( &urg_fs.data );
				gdrawer.setOBPoint( &obp_fs.data );
				gdrawer.drawGraph( );


			} else {
				usleepSSM( dT * 1000 );
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
	fprintf( stderr, "\t$ %s -i -l -n 0 -t 100\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-i | --intensity           : Use Intensity data. (default=%d)\n", flag_intensity );
	printf( "\t-l | --laser               : Draw laser. (default=%d)\n", flag_laser );
	printf( "\t-n | --number       NUMBRE : set sensor ID number. (default=%d)\n", sensor_id );
	printf( "\t-t | --s_time       TIME   : Wait time (defautl=%dms)\n", dT );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "number", 1, 0, 'n' },
		{ "s_time", 1, 0, 't'},
		{ "intensity", 1, 0, 'i' },
		{ "laser", 1, 0, 'l' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "n:t:ilh", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'n':
			sensor_id = atoi( optarg );
			break;
		case 't':
			dT = atoi( optarg );    
			break;
		case 'i':
			flag_intensity = true;
			break;
		case 'l':
			flag_laser = true;
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
	endSSM( );
	Gprint( "end\n" );
}
