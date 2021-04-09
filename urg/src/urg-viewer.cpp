/*
 * urg-viewer.cpp
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include "urg.hpp"
#include "utility.hpp"
#include "GraphDrawerURG.hpp"

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) (x)
#else
#define DEBUG_PRINT(x)
#endif

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );
static int printProperty( urg_property *urg );

static int sensor_id = 0;
static bool flag_intensity = false;
static bool flag_laser = false;
static unsigned int dT = 100;	// 100ms

// SSM valiant
static SSMApi <urg_fs, urg_property> *Urg;

int main( int aArgc, char *aArgv[] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi <urg_fs, urg_property> urg_fs( URG_SNAME, sensor_id );
		Urg = &urg_fs;
		
	try {
		setupSSM( );
		setSigInt( );

		GraphDrawerURG gdrawer;
		gdrawer.setParameter( flag_laser, flag_intensity, &urg_fs.property );
		gdrawer.initGnuplot( );				// gnuplot初期化
		gdrawer.setRange( 20, 20 );

		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_URG			0

		Gprint( "\nStart urg-viewer\n" );
		while( !gShutOff ){
			update[ INDEX_URG ] = false;
			// get latest data for INDEX_URG
			if( update_id[ INDEX_URG ] < getTID_top( urg_fs.getSSMId( ) ) ){
				urg_fs.readLast(  );
				update[ INDEX_URG ] = true; // 最新情報を読み込む
				update_id[ INDEX_URG ] = urg_fs.timeId;
			} else {
				update[ INDEX_URG ] = false;
			}
			if( update[ INDEX_URG ] ){

				DEBUG_PRINT( printf( "size = %d\n", urg_fs.data.size ) );
				gdrawer.setScan( &urg_fs.data );
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

	// get urg_fs.property
	std::cerr << "\n" << "get urg_fs.property ... ";
	if( !Urg->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to get urg_fs.property.");
	else
		std::cerr << "OK.\n";
		
	// urgを開く
	std::cerr << "open urg ... ";
	if( !Urg->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open urg on ssm.\n" );
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	Urg->release( );
	endSSM( );
	Gprint( "\nend\n" );
}

