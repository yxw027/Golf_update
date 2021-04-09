/*
 * WP-viewer
 * 
 * Create : 2019.09.30
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

#include "wp.hpp"
#include "wp-mgr.hpp"
#include "GraphDrawerWP.hpp"
#include "utility.hpp"
#include "config.hpp"

//#define TIME_LAPSE

static int gShutOff = 0;
static void setSigInt(  );
static int printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 1000;
static char filename[ STRLEN ] = "../../param/WP_tmp.dat";
static bool flag_wpfile_config = true;
static bool flag_anima = false;
static SSMApi <config, config_property> *CONF;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;
		
	try {
		setupSSM( );
		setSigInt( );

		GraphDrawerWP drawer;			// gnuplotによる描画
		drawer.setParameters( config.property, flag_wpfile_config, filename );
		drawer.initGnuplot( );				// gnuplot初期化
		drawer.setGrid( );
		drawer.setAspectRatio( -0.9 );		// x軸とy軸の比（負にすると中身が一定）

		Gprint( "Start WP-viewer\n" );
		while( !gShutOff ){

			if( flag_anima ){
				drawer.updateWP( );
			}
			drawer.drawGraph( );
			usleepSSM( dT * 1000 );

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
static int printShortHelp( const char *programName )
{
	fputs( "HELP\n", stderr );
	fprintf( stderr, "\t$ %s [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -t 100 -f ../param/WP.dat \n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-f | --file      WP     : Input WP file (default=%s)\n", filename );
	printf( "\t-a | --anima     ANIMA  : Animation mode (default=%d)\n", flag_anima );
	printf( "\t-t | --s_time    TIME   : Wait time (defautl=%dms)\n", dT );
	printf( "\n" );
	return EXIT_SUCCESS;
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "s_time", 1, 0, 't'},
		{ "file", 1, 0, 'f' },
		{ "anima", 1, 0, 'a' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "t:f:ah", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 't':
			dT = atoi( optarg );
			break;
		case 'f':
			sprintf( filename, "%s", optarg );
			flag_wpfile_config = false;
			break;
		case 'a':
			flag_anima = true;
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
	endSSM( );
	Gprint( "end\n" );
}

