/*
 * control-log2txtviewer
 * 
 * Create : 2019.09.23
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
#include "config.hpp"
//#include "TrajectoryDrawer.hpp"
#include "utility.hpp"
#include "control.hpp"
#include "log2txt.hpp"

static int gShutOff = 0;
static void setSigInt(  );
static int printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 5;
static bool flag_save_control = true;
static bool flag_save_localizer = false;

static SSMApi <config, config_property> *CONF;
static SSMApi <localizer> *LOCAL;
static SSMApi <control> *CNTL;
int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;
	SSMApi<localizer> localizer( LOCALIZER_SNAME, 0 );
		LOCAL = &localizer;
	SSMApi <control> control( CONTROL_SNAME, 0 );
		CNTL = &control;
		
	try {
		setupSSM( );
		setSigInt( );
		Log2Txt_Control savelfile_control;
		Log2Txt_Localizer savelfile_localizer;
		
		if( flag_save_localizer ){
			savelfile_localizer.openSaveFile( "log_localizer.dat" );
		}
		if( flag_save_control ){
			savelfile_control.openSaveFile( "log_control.dat" );
		}
			
		bool update[ 3 ] = { false };
		SSM_tid update_id[ 3 ] = { -1 };
#define INDEX_LOCALIZER		0
#define INDEX_WP			1
#define INDEX_CONTROL		2

		Gprint( "Start control-log2txt\n" );
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
			if( ( update_id[ INDEX_CONTROL ] < getTID_top( control.getSSMId( ) ) ) && flag_save_control ){
				control.readLast(  );
				update[ INDEX_CONTROL ] = true; // 最新情報を読み込む
				update_id[ INDEX_CONTROL ] = control.timeId;
			} else {
				update[ INDEX_CONTROL ] = false;
			}
						
			if( update[ INDEX_CONTROL ] ){
				if( flag_save_control ){
					savelfile_control.log2txt( &control.data, control.time );
				}
			}
			
			if( update[ INDEX_LOCALIZER ] ){
				if( flag_save_localizer ){
					savelfile_localizer.log2txt( &localizer.data, localizer.time );
				}
				
			} else {
				usleepSSM( dT * 1000 );
			}
		}
		
		if( flag_save_localizer ){
			savelfile_localizer.closeSaveFile( );
		}
		if( flag_save_control ){
			savelfile_control.closeSaveFile( );
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
	fprintf( stderr, "\t$ %s -t 100 -s\n", programName );
	fputs( "OPTION\n", stderr );
	printf( "\t-l | --localizer SAVE   : Save localizer data to log_localizer.dat (default=%d)\n", flag_save_localizer );
	printf( "\t-c | --control   SAVE   : Not save control data to log_control.dat (default=%d)\n", flag_save_control );
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
			flag_save_control = false;
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
	LOCAL->release( );
	if( flag_save_control ) CNTL->release( );
	endSSM( );
	Gprint( "end\n" );
}

