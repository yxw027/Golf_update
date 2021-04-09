#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerPose.hpp"
#include "localizer.hpp"
#include "utility.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 100;	// 100ms
static int keyframeSkip = 0;		// キーフレーム間隔
static bool flag_save = false;

static SSMApi <config, config_property> *Conf;
static SSMApi <localizer> *LOCAL;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		Conf = &config;
	SSMApi <localizer> localizer( LOCALIZER_SNAME, 0 );
		LOCAL = &localizer;

	try {
		setupSSM( );
		setSigInt( );
		
		unsigned long counter = 0;
		GraphDrawerPose drawer;
		drawer.setParameters( &config.property, flag_save );
		drawer.initGnuplot( );
				
		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_LOCAL		0

		Gprint( "Start localizer-viewer\n" );		
		while( !gShutOff ){
			update[ INDEX_LOCAL ] = false;
			// get latest data for INDEX_LOCAL
			if( update_id[ INDEX_LOCAL ] < getTID_top( localizer.getSSMId( ) ) ){
				localizer.readLast(  );
				update[ INDEX_LOCAL ] = true; // 最新情報を読み込む
				update_id[ INDEX_LOCAL ] = localizer.timeId;
			} else {
				update[ INDEX_LOCAL ] = false;
			}
			
			if( update[ INDEX_LOCAL ] ){

				drawer.setPose( &localizer.data, localizer.time );
				if( counter >= keyframeSkip ){			// キーフレームのときだけ行う
					drawer.drawGraph( );
					counter = 0;
				}
				counter++;

			} else {
				usleep( dT * 1000 );
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
	fprintf( stderr, "\t$ %s -t 100\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-t | --s_time       TIME   : Sampling time (defautl=%dms)\n", dT );
	printf( "\t-s | --save         SAVE   : Save log data to file (default=%d)\n", flag_save );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "s_time", 1, 0, 't' },
		{ "save", 1, 0, 's' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "t:sh", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
   		case 't':
			dT = atoi( optarg );    
			break;
		case 's':
			flag_save = true;
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
			
	// configのpropertyを取得
	std::cerr << "get config ... ";
	if( !Conf->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to config get.");
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
	LOCAL->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
