/*
 *  imu-handler.cpp
 *
 * Create: 2018/07/12
 * Update: 2019/01/11
 * Update: 2021/03/16
 * Author: T. Hasegawa
 */
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <fcntl.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <stdexcept>

#include <ssm.hpp>
#include "imu.hpp"
#include "9axisIMU.hpp"
#include "utility.hpp"
#include "framework.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static char device_name[ STRLEN ] = "/dev/ttyACM0";
static unsigned int Time4Initial = 5;
 
static SSMApi<imu_fs, imu_property> *IMU;

int main( int aArgc, char **aArgv )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	IMU_CLASS imu9axis;
	imu9axis.openIMU( device_name, Time4Initial );

	SSMApi<imu_fs, imu_property> imu_fs( IMU_SNAME, 0 );
		IMU = &imu_fs;
	
	try{
		imu9axis.getIMUproperty( &IMU->property );
		setupSSM( );
		setSigInt( );

		Gprint( "\nStart imu-handler\n\n" );
		while( !gShutOff ){
			
			bool ret = imu9axis.receiveIMU( );
			if( !ret ) continue;
			
			// 相補フィルターの計算
			imu9axis.ComplementaryFilter( );
			imu9axis.getIMUdata( &imu_fs.data );
			imu_fs.write( );

		}
	}
	catch( std::runtime_error const & error ){
		std::cout << error.what( ) << std::endl;
	}
	catch( ... ){
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}
	imu9axis.closeIMU( );
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
	fprintf( stderr, "\t$ %s -d %s\n", programName, device_name );
	fputs( "OPTION\n", stderr );
	printf( "\t-d | --device     DEVICE : set input device (default=%s)\n", device_name );
	printf( "\t-t | --time       TIME   : set waiting time for initialization (default=%d[s])\n", Time4Initial );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
        { "device", 1, 0, 'd' },
        { "time", 1, 0, 't' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "d:t:h", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'd':
			sprintf( device_name, "%s", optarg );
			break;
		case 't':
			Time4Initial = atoi( optarg );
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
	unsigned int dT = 5;
		
	std::cerr << "initializing ssm ... ";
	if( !initSSM( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to initialize ssm.");
	else
		std::cerr << "OK.\n";
			
	// create imu_fs
	std::cerr << "create imu_fs ... ";
	if( !IMU->create( 1, ( double )dT/1000.0 ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create imu on ssm.\n");
	else
		std::cerr << "OK.\n";
		
	// set imu_fs.property
	std::cerr << "set imu_fs.property ... ";
	if( !IMU->setProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to ssm open.");
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	IMU->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
