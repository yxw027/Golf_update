#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include "utility.hpp"
#include "OMcntl.hpp"
#include "GraphDrawerHandle.hpp"

#define TIME_WIDTH 15 // sec

using namespace std;

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 10;
static bool flag_save = false;

static SSMApi<OMcntl> *OMotor;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi<OMcntl> handle( OM_CNTL_SNAME, 0 );
		OMotor = &handle;

	try {
		setupSSM( );
		setSigInt( );

		GraphDrawerHandle gdrawer;		// gnuplotによる描画
		gdrawer.setParameters( _HANDLE, flag_save );
		gdrawer.initGnuplot( );				// gnuplot初期化
				
		Gprint( "\nStart steering-viewer\n" );

		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_HANDLE			0
	
		while( !gShutOff ){
			update[ INDEX_HANDLE ] = false;
			// get latest data for INDEX_HANDLE
			if( update_id[ INDEX_HANDLE ] < getTID_top( handle.getSSMId( ) ) ){
				handle.readLast(  );
				update[ INDEX_HANDLE ] = true; // 最新情報を読み込む
				update_id[ INDEX_HANDLE ] = handle.timeId;
			} else {
				update[ INDEX_HANDLE ] = false;
			}
			
			if( update[ INDEX_HANDLE ] ){
				
				gdrawer.setHandleInfo( &handle.data, handle.time );
				gdrawer.drawGraph( );

			} else {
				usleep( dT * 1000 );
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
	fprintf( stderr, "\t$ %s -s\n", programName );
	printf( "OPTION\n" );
	printf( "\t-s | --save         SAVE   : Save log data to file (default=%d)\n", flag_save );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "save", 1, 0, 's' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "sh", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){;
		case 's':
			flag_save = true;
			break;
		case 'h':
			printShortHelp( aArgv[0] );
			return false;
			break;
		default:
			fprintf( stderr, "help : %s -h\n", aArgv[0] );
			return false	;
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

	// OMcntl を開く
	std::cerr << "open  OMcntl ... ";
	if( !OMotor->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open OMcntl on ssm.\n" );
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	OMotor->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
