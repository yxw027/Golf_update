#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <linux/input.h>
#include <linux/joystick.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <ssm.hpp>
#include "ModbusRTU.hpp"
#include "OMcntl.hpp"
#include "utility.hpp"
#include "operateJoystick.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static char device_name[ 128 ] = "/dev/ttyUSB0";
static unsigned int dT = 20;	// 20ms
static SSMApi<OMcntl> *OMotor;
static SSMApi <config, config_property> *CONF;

class recordMotor : public operateJoystick
{
public:
	recordMotor( void ) { }
	~recordMotor( void ) { }
	
	void Start( void ){ operateJoystick::Start( ); }
	void Free( void ){ operateJoystick::Free( ); }
	void setZeroPosition( void ){ operateJoystick::setZeroPosition( ); }
	bool Record( void )
	{
		operateJoystick::getMotorInfo( _ACCEL );
		operateJoystick::getMotorInfo( _HANDLE );
		operateJoystick::getMotorInfo( _LEVER );
		return true;
	}
};
int main( int aArgc, char **aArgv )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi<OMcntl> OMcntl( OM_CNTL_SNAME, 0 );
		OMotor = &OMcntl;
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;
		
	recordMotor rmotor;
	rmotor.openOMController( device_name );	// モータードライバと接続
	
	try {
		setupSSM( );
		setSigInt( );

		rmotor.setConfig( config.property );

		// ゼロ点設定
		rmotor.setZeroPosition( );
		fprintf( stderr, "Hit any key, to start the recorder program.\n" );
		while( !kbhit( ) ){	}
		// フリーモード設定
		rmotor.Free( );

		Gprint( "\nStart recordOperation\n" );
		while( !gShutOff ){
			
			bool ret = rmotor.Record( );
			if( ret ){
				OMcntl.data = rmotor.getOMcntl( );
				OMcntl.write( );
			}
			usleep( dT * 1000 );
		}
	}
	catch( std::runtime_error const & error ){
		std::cout << error.what() << std::endl;
	}
	catch( ... ){
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}
	
	rmotor.closeOMController( );
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
	printf( "\nHELP\n" );
	printf( "\t$ %s [   options    ]\n", programName );
	printf( "\t$ %s -d %s\n", programName, device_name );
	printf( "OPTION\n" );
	printf( "\t-d | --device     DEVICE   : set input device (default=%s)\n", device_name );
	printf( "\t-t | --s_time     TIME     : Sampling time (defautl=%dms)\n", dT );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	bool flag_device = false;
	int opt, optIndex = 0;
	struct option longOpt[] = {
		{ "s_time", 1, 0, 't'},
		{ "device", 1, 0, 'd' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "d:t:h", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'd':
			sprintf( device_name, "%s", optarg );
			flag_device = true;
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

	// OMcntl を開く
	std::cerr << "create OMcntl ... ";
	if( !OMotor->create( 1, ( double )100/1000.0 ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create OMcntl on ssm.\n" );
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
	OMotor->release( );
	CONF->release( );
	endSSM( );
	Gprint( "\nend\n\n" );
}

