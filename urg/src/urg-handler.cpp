/*
 * LiDAR for Hokuyo Co. 
 * 
 * Date    : 2017.03.10
 * Updated : 2018.05.02
 * Updated : 2018.12.12
 * Updated : 2019.03.31
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
#include <scip2awd.h>
#include "utility.hpp"
#include "urg.hpp"
#include "lidar.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 20;	// 20ms
static int sensor_id = 0;
static bool flag_intensity = true;
static char device_name[ STRLEN ] = "/dev/ttyACM0";
static SSMApi <urg_fs, urg_property> *Urg;
	
int main( int aArgc, char **aArgv )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <urg_fs, urg_property> urg_fs( URG_SNAME, sensor_id );
		Urg = &urg_fs;

	lidar_Hokuyo lidar;
	lidar.initialize( device_name, flag_intensity );
	lidar.getProperty( &urg_fs.property );

	try {

		setupSSM( );
		setSigInt( );
		
		Gprint( "\nstart urg-handler\n" );
		while( !gShutOff ){

			bool ret = lidar.getPointCloudData( &urg_fs.data );
			if( ret ) urg_fs.write( );
			
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
static void printShortHelp( const char *programName )
{
	fputs( "HELP\n", stderr );
	fprintf( stderr, "\t$ %s DEVICE_PATH [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -d 192.168.0.10:10940  Use URG on <IP:Port>\n", programName );
	fprintf( stderr, "\t$ %s -d /dev/ttyACM*        Use URG on </dev/ttyACM*>\n", programName );
	fprintf( stderr, "\t$ %s -d /dev/ttyACM* -n 0   Out SSM on <sensor_id=%d>\n", programName, sensor_id );
	printf( "OPTION\n" );
	printf( "\t-d | --device              : set device port\n" );
	printf( "\t-i | --intensity           : NOT Use Intensity data. (default=%d)\n", flag_intensity );
	printf( "\t-n | --number       NUMBRE : set sensor ID number. (default=%d)\n", sensor_id );
	printf( "\t-t | --s_time       TIME   : Wait time (defautl=%dms)\n", dT );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "device", 1, 0, 'd'},
		{ "number", 1, 0, 'n' },
		{ "s_time", 1, 0, 't'},
		{ "intensity", 1, 0, 'i' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "d:n:t:ih", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'd':
			sprintf( device_name, "%s", optarg );
			break;
		case 'n':
			sensor_id = atoi( optarg );
			break;
		case 't':
			dT = atoi( optarg );    
			break;
		case 'i':
			flag_intensity = false;
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
	std::cerr << "\n" << "initializing ssm ... ";
	if( !initSSM() )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to initialize ssm.");
	else
		std::cerr << "OK.\n";
		
	// create urg_fs
	std::cerr << "create urg_fs ... ";
	if( !Urg->create( 1, ( double )dT/1000.0 ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create urg_fs on ssm.\n");
	else
		std::cerr << "OK.\n";

	//inpProperty( &param, &ver, &Urg->property );

	// set urg_fs.property
	std::cerr << "\n" << "set urg_fs.property ... ";
	if( !Urg->setProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to ssm open.");
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	Urg->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
