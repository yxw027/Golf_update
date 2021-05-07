#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include "utility.hpp"
#include "imu.hpp"
#include "GraphDrawerIMU.hpp"

#define TIME_WIDTH 15 // sec

using namespace std;

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int wait_time = 100;
static int graph_mode = 1;
static bool flag_save = false;

static SSMApi<imu_fs, imu_property> *IMU;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi<imu_fs, imu_property> imu_fs( IMU_SNAME, 0 );
		IMU = &imu_fs;

	try {
		setupSSM( );
		setSigInt( );

		GraphDrawerIMU gdrawer;	// gnuplotによる描画
		gdrawer.setParameters( graph_mode, flag_save );
		gdrawer.printProperty( &imu_fs.property );
		gdrawer.initGnuplot( );				// gnuplot初期化
				
		Gprint( "\nStart imu-viewer\n" );

		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_IMU			0
	
		while( !gShutOff ){
			update[ INDEX_IMU ] = false;
			// get latest data for INDEX_IMU
			if( update_id[ INDEX_IMU ] < getTID_top( imu_fs.getSSMId( ) ) ){
				imu_fs.readLast(  );
				update[ INDEX_IMU ] = true; // 最新情報を読み込む
				update_id[ INDEX_IMU ] = imu_fs.timeId;
			} else {
				update[ INDEX_IMU ] = false;
			}
			
			if( update[ INDEX_IMU ] ){
				
				gdrawer.setIMU( &imu_fs.data, imu_fs.time );
				gdrawer.drawGraph( );

			} else {
				usleepSSM( wait_time * 1000 );
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
	fprintf( stderr, "\t$ %s -m 5 -s\n", programName );
	printf( "OPTION\n" );
	printf( "\t-m | --mode                : Change display mode. 1:AngVel, 2:accel, 3:mag, 4:temp., 5:estAng (default=%d)\n", graph_mode );
	printf( "\t-s | --save                : Save log data (1:Save default=%d)\n", flag_save );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "mode", 1, 0, 'm' },
		{ "save", 1, 0, 's' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "m:sh", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){;
		case 'm':
			graph_mode = atoi( optarg ); 
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
			return false	;
			break;
		}
	}	
	return true;
}
static void setupSSM( void )
{
	unsigned int dT = 5;
		
	std::cerr << "initializing ssm ... ";
	if( !initSSM( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to initialize ssm.");
	else
		std::cerr << "OK.\n";
	
	// get imu_fs.property
	std::cerr << "get imu_fs.property ... ";
	if( !IMU->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to get imu_fs.property.");
	else
		std::cerr << "OK.\n";
		
	// imu_fsを開く
	std::cerr << "open  imu_fs... ";
	if( !IMU->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open imu_fs on ssm.\n" );
	else
		std::cerr << "OK.\n\n";
}
static void Terminate( void )
{
	IMU->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
