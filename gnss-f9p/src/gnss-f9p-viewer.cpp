/*
 * gnss-f9p-viewer.cpp
 * Create on 2017.11.24
 * Update : 2019.08.01
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
#include "gnss-f9p.hpp"
#include "GraphDrawerGNSS-f9p.hpp"
#include "utility.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static bool flag_save = false;
static unsigned int dT = 100;	// 100ms
static SSMApi <config, config_property> *Conf;
static SSMApi <rtk_gnss_f9p> *GNSS;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
		
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		Conf = &config;
	SSMApi <rtk_gnss_f9p> rtk_gnss( SNAME_GNSS_F9P, 0 );
		GNSS = &rtk_gnss;

	try {
		setupSSM( );
		setSigInt( );
		
		GraphDrawerGNSS drawer;
		drawer.setParameters( &config.property, flag_save );
		drawer.initGnuplot( );
		
		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_RTK_GNSS	0
		
		Gprint( "\nstart gnss-f9p-viewer\n" );
		while( !gShutOff ){
			update[ INDEX_RTK_GNSS ] =  false;
			// get latest data for INDEX_RTK_GNSS
			if( update_id[ INDEX_RTK_GNSS ] < getTID_top( rtk_gnss.getSSMId( ) ) ){
				rtk_gnss.readLast(  );
				update[ INDEX_RTK_GNSS ] = true; // 最新情報を読み込む
				update_id[ INDEX_RTK_GNSS ] = rtk_gnss.timeId;
			} else {
				update[ INDEX_RTK_GNSS ] = false;
			}
			
			if( update[ INDEX_RTK_GNSS ] ){
				
				drawer.setPose( &rtk_gnss.data, rtk_gnss.time );
				drawer.drawGraph( );

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
	fprintf( stderr, "\t$ %s\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-t | --s_time       TIME   : Sampling time (defautl=%dms)\n", dT );
	printf( "\t-s | --save         SAVE   : Save log data to file (default=%d)\n", flag_save );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "save", 1, 0, 's' },
		{ "s_time", 1, 0, 't' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "t:sh", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 's':
			flag_save = true;
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
	
	// configのpropertyを取得
	std::cerr << "get config ... ";
	if( !Conf->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to config get.");
	else
		std::cerr << "OK.\n";
		
	// rtk_gnssを開く
	std::cerr << "open rtk_gnss ... ";
	if( !GNSS->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open rtk_gnss on ssm.\n" );
	else
		std::cerr << "OK.\n";
		
}
static void Terminate( void )
{
	GNSS->release( );
	Conf->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
