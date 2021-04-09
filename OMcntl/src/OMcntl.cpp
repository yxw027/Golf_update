/*
 * OMcntl.cpp
 * 芝刈機アクチュエータ用サンプルプログラム
 *
 * Data: 2019/06/11
 * Update : 2021/03/26
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
#include "ModbusRTU.hpp"
#include "config.hpp"
#include "utility.hpp"
#include "framework.hpp"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT( x ) ( x )
#else
#define DEBUG_PRINT( x )
#endif

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 100;
static char device_name[ STRLEN ] = "/dev/ttyUSB0";
static SSMApi <config, config_property> *CONF;

//#ifdef MODBUS_UPDATE
//static ModbusRTU_TempRef modbus;
//#else
//static ModbusRTU modbus;
//#endif
static ModbusRTU_CLASS modbus;
static int id = 1;

//static bool kbhit( void );
//static void Gprint( const char *str );
//static void Rprint( const char *str );
static void sendCommand( int key );
static void Usage( void );
int main( int aArgc, char **aArgv )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;

	modbus.openController( device_name );
	
	try{
		setupSSM( );
		setSigInt( );

		Gprint( "Start OMcntl <Sample Program>\n" );
		Usage( );
		
		modbus.setConfig( config.property );
		
		while( !gShutOff ){
			
			if( kbhit( ) ){
				int keyboard = getchar( );
				sendCommand( keyboard );
			}
			usleep( dT * 1000 );
		}
	}
	catch( std::runtime_error const & error ){
		std::cout << error.what( ) << std::endl;
	}
	catch( ... ){
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
	fprintf( stderr, "\t$ %s -d %s\n", programName, device_name );
	fputs( "OPTION\n", stderr );
	printf( "\t-d | --device       DEVICE : set input device (default=%s)\n", device_name );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	bool flag_device = false;
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
        { "device", 1, 0, 'd' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "d:h", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'd':
			sprintf( device_name, "%s", optarg );
			flag_device = true;
			break;
		case 'h':
			printShortHelp( aArgv[0] );
			Usage( );
			return false;
			break;
		default:
			fprintf( stderr, "help : %s -h\n", aArgv[0] );
			return false;
			break;
		}
	}

	if( !flag_device ){
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]:MUST set the input device.\n");
		return false;
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
	endSSM( );
	Gprint( "end\n" );
	modbus.closeController( );
}
static void Usage( void )
{
	printf( "USAGE\n" );
	printf( "Hit the following key to send a command.\n" );
//	printf( "<ID:%d>\n", id );
	printf( "\t c : Chnage ID (Present ID=%d)\n", id );
	printf( "\t p : Position\n" );
	printf( "\t a : Angle\n" );
	printf( "\t r : Read Register\n" );
//	printf( "\t m : Get Present Step\n" );
	printf( "\t s : Start\n" );
	printf( "\t f : Free\n" );
	printf( "\t z : Return Zero Position\n" );
	printf( "\t n : Set Zero Position\n" );
	printf( "\t y : Check Alarm\n" );
	printf( "\t x : Alarm Reset\n" );
	printf( " > " );
}
static void sendCommand( int key )
{
	if( key == 'c' ){
		Gprint( "\nChnage ID\n" );
		if( id == _ACCEL ) id = _HANDLE;
		else if( id == _HANDLE ) id = _LEVER;
		else if( id == _LEVER ) id = _ACCEL;
		Usage( );
		
	} else if( key == 'p' ){
		Gprint( "\nPosition\n" );
		Usage( );
		if( id == _ACCEL || id == _LEVER ){
			modbus.sendPos( id, 67 );//from 52 to 85(n:67) //from 40 to 60 (n:55)
			fprintf( stderr, "Pos=%f, Tau=%f\n", modbus.getPosition( id ), modbus.getTorque( id ) );
		} else
			fprintf( stderr, "Error : Need to change ID to 1 or 3.\n" );
			
	} else if( key == 'a' ){
		Gprint( "\nAngle\n" );
		Usage( );
		if( id == _HANDLE ){
			modbus.sendAng( id, -90 );	// -90度
			fprintf( stderr, "Ang=%f, Tau=%f\n", modbus.getAngle( ), modbus.getTorque( id ) );
		} else
			fprintf( stderr, "Error : Need to change ID to 2.\n" );
			
	} else if( key == 'r' ){
		Gprint( "\nRead Register\n" );
		Usage( );
		modbus.readRegistor( id );
	
	} else if( key == 's' ){
		Gprint( "\nStart\n" );
		Usage( );
		modbus.sendStart( id );

	} else if( key == 'f' ){
		Gprint( "\nFree\n" );
		Usage( );
		modbus.sendFree( id );
		
	} else if( key == 'z' ){
		Gprint( "\nReturn Zero Position\n" );
		Usage( );
		modbus.returnZero( id );
		modbus.resetZeroCommand( id );
		if( id == 2 ) modbus.resetAngle( );

	} else if( key == 'x' ){
		Gprint( "\nAlarm Reset\n" );
		Usage( );
		modbus.sendALM_RST( id );

	} else if( key == 'y' ){
		Gprint( "\nCheck Alarm\n" );
		Usage( );
		modbus.chkALM( id );
	
	} else if( key == 'n' ){
		Gprint( "\nSet Zero Position\n" );
		Usage( );
		modbus.setZeroPosition( id );
	
	} else {
		Rprint( "\nError" );
		printf( ":Hit the following command.\n" );
		Usage( );
	}
}

