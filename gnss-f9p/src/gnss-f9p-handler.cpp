//-------------------------------------------------
// gnss-f9p-handler.cpp
// T.Hasegawa
// Since: 2017-10-29
// Update: 2019/07/09
// Update: 2019/08/16
// Update: 2021/03/16
//-------------------------------------------------
#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <deque>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <netinet/ip.h> 
#include <sys/types.h>
#include <ssm.hpp>
#include <errno.h>
#include <arpa/inet.h>
#include "gnss-f9p.hpp"
#include "rtknavi-f9p.hpp"
#include "utility.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static int printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static char device_name[ STRLEN ] = "/dev/ttyACM0";	//"/dev/ttyUSB0";
static unsigned int dT = 100;	// 100ms
static bool flag_check_data = false;
static int coordinator_id = 9;

static SSMApi <rtk_gnss_f9p> *GNSS;

int main( int aArgc, char **aArgv )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <rtk_gnss_f9p> rtk_gnss( SNAME_GNSS_F9P, 0 );
		GNSS = &rtk_gnss;
	
	RTKNAVI_F9P rtknavi;
	
	try {
		setupSSM( );
		setSigInt( );

		// 直角平面座標系の設定
		rtknavi.setCoodinatorID( coordinator_id );
		// 通信確立
		rtknavi.openGNSSReceiver( device_name );

		Gprint( "Start gnss-f9p-handler\n" );
	
		while( !gShutOff ){
			
			// GPSから受信
			bool ret = rtknavi.decodeRTKGNSS( &rtk_gnss.data );
			if( ret ){
				rtk_gnss.write( );
				if( flag_check_data ) rtknavi.printRTKGNSS( &rtk_gnss.data );
			} else {
				if( flag_check_data ) cerr << "No Data\n";
			}
			usleep( dT * 1000 );

		}
		rtknavi.closeGNSSReceiver( );
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
	fprintf( stderr, "\t$ %s DEVICE_PATH [   options    ]\n", programName );
	fprintf( stderr, "\t$ %s -d %s  Use RTKLIB on <IP:Port>\n", programName, device_name );
	fprintf( stderr, "\t$ %s -t %d\n", programName, dT );
	fputs( "OPTION\n", stderr );
	printf( "\t-t | --s_time   TIME     : Sampling time (defautl=%dms)\n", dT );
	printf( "\t-d | --device   DEVICE   : Set device (defautl=%s)\n", device_name );
	printf( "\t-i | --cid      ID       : Set coordinator ID (defautl=%d)\n", coordinator_id );
	printf( "\t-c | --check    CHECK    : Check gnss data (defautl=%d, (1:Chk,0:NotChk))\n", flag_check_data );
	return EXIT_SUCCESS;
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "s_time", 1, 0, 't' },
		{ "device", 1, 0, 'd' },
		{ "cid", 1, 0, 'i' },
		{ "check", 1, 0, 'c' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};
	
	while( ( opt = getopt_long( aArgc, aArgv, "d:t:i:ch", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 't':
			dT = atoi( optarg );    
			break;
		case 'i':
			coordinator_id = atoi( optarg );    
			break;
		case 'c':
			flag_check_data = true;    
			break;
		case 'd':
			sprintf( device_name, "%s", optarg );
			break;
		case 'h':
			printShortHelp( aArgv[ 0 ] );
			return false;
			break;
		default:
			fprintf( stderr, "help : %s -h\n", aArgv[ 0 ] );
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
		std::cerr << "OK.\n\n";

	// Open rtk_gnss
	std::cerr << "create rtk_gnss ... ";
	if( !GNSS->create( 2, ( double )dT/1000.0 ) )
		std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create rtk_gnss on ssm.\n" );
	else
		std::cerr << "OK.\n";
}
static void Terminate( void )
{
	GNSS->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
