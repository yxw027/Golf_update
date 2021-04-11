/*
 * Create WP file, while a mobile robot is moved by manual, or log is played
 * 
 * Date : 2018.04.10
 * Update : 2019.09.03
 * Update : 2021.04.11
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
#include "localizer.hpp"
#include "utility.hpp"
#include "config.hpp"
#include "GraphDrawerWP.hpp"
#include "createWP.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 100;	// 100ms
static char path[ STRLEN ] = "../data";
SSMApi<localizer> *Local;
SSMApi <config, config_property> *Conf;
	
int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi<localizer> localizer( LOCALIZER_SNAME, 0 );
		Local = &localizer;
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		Conf = &config;
	
	try {
		setupSSM( );
		setSigInt( );

		GraphDrawerWP drawer;
		drawer.setParameter( &config.property, path );
		drawer.initGnuplot( );

		Gprint( "Start createWPfile\n" );

		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { 0 };
#define INDEX_LOCALIZER		0
		
		while( !gShutOff ){
			update[ INDEX_LOCALIZER ] = false;
			// get latest data for INDEX_LOCALIZER
			if( update_id[ INDEX_LOCALIZER ] < getTID_top( localizer.getSSMId( ) ) ){
				localizer.readLast(  );
				update[ INDEX_LOCALIZER ] = true; // 最新情報を読み込む
				update_id[ INDEX_LOCALIZER ] = localizer.timeId;
			} else {
				update[ INDEX_LOCALIZER ] = false;
			}

			if( update[ INDEX_LOCALIZER ] ){
				
				drawer.setPose( &localizer.data );
				drawer.drawGraph( );
				
			} else {
				usleepSSM( dT * 1000 );
			}
		}
		drawer.writeSaveFile( );
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
static void setSigInt( void )
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
	fprintf( stderr, "\t$ %s -p ../data\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-p | --path   	  PATH     : Path of directory for output (default=%s)\n", path );
	printf( "\t-t | --s_time      TIME     : Wait time (defautl=%dms)\n", dT );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "s_time", 1, 0, 't' },
		{ "path", 1, 0, 'p' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "p:t:p:h", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'p':
			strcpy( path, optarg );
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
	if( !Local->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open localizer on ssm.\n" );
	else
		std::cerr << "OK.\n";

	// configのpropertyを取得
	std::cerr << "get config ... ";
	if( !Conf->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to config get.");
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	Local->release( );
	Conf->release( );
	endSSM( );
	Gprint( "end" );
}
