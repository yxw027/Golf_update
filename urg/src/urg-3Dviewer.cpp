#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <unistd.h>
#include <ssm.hpp>
#include "urg.hpp"
#include "utility.hpp"
#include "GraphDrawerPCD.hpp"

static int gShutOff = 0;
static void setSigInt( void );

static unsigned int dT = 10;
static int sensor_id = 0;
static bool flag_save = false;
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static SSMApi <urg_fs, urg_property> *Urg;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <urg_fs, urg_property> urg_fs( URG_SNAME, sensor_id );
		Urg = &urg_fs;
		
	try {
		setupSSM( );
		setSigInt( );

		GraphDrawerPCD gdrawer;		// PCLによる描画
		gdrawer.initialize( false, false );		// visualizer初期化

		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_URG			0

		printf( "\nStart urg-3Dviewer\n" );
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
			
				gdrawer.setScan( &urg_fs.data );
				bool ret = gdrawer.drawGraph( );
				if( !ret ) break;
			
				if( flag_save ) gdrawer.savePCD( );
						
			} else {
				usleepSSM( dT * 1000 );
			}
			
		}
	}
	catch ( std::runtime_error const & error ){
		std::cout << error.what() << std::endl;
	}
	catch ( ... ){
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
	fprintf( stderr, "\t$ %s -t %d -s\n", programName, dT );
	printf( "OPTION\n" );
	printf( "\t-s | --save     SAVE   : Save pcd. (defalut=%d)\n", flag_save );
	printf( "\t-t | --stime    TIME   : Sampling time. (defalut=%d[ms])\n", dT );
	printf( "\n" );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "save", 1, 0, 's' },
		{ "stime", 1, 0, 't' },
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
static int printProperty( urg_property *urg )
{
	printf( "\n<URG INFO>\n" );
	printf( "vernder  : %s\n", urg->vender );
	printf( "product  : %s\n", urg->product );
	printf( "firmware : %s\n", urg->firmware );
	printf( "protocol : %s\n", urg->protocol );
	printf( "serial no: %s\n\n", urg->serialno );

	printf( "<%s>\n", urg->model );
	printf( "%5d # MINIMUM DISTANCE [mm]\n", urg->dist_min );
	printf( "%5d # MAXIMUM DISTANCE [mm]\n", urg->dist_max );
	printf( "%5d # STEP RESOLUTION\n", urg->step_resolution );
	printf( "%5d # MINIMUM STEP\n", urg->step_min );
	printf( "%5d # MAXIMUM STEP\n", urg->step_max );
	printf( "%5d # STEP IN FRONT OF URG\n", urg->step_front );
	printf( "%5d # REVOLUTION [rpm]\n", urg->revolution );
	
	return EXIT_SUCCESS;
}

