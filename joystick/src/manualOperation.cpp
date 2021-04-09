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
#include <joystick.hpp>
#include "ModbusRTU.hpp"
#include "OMcntl.hpp"
#include "utility.hpp"
#include "operateJoystick.hpp"
#include "config.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static unsigned int dT = 1;	// 1ms
static char device_name[ 128 ] = "/dev/ttyUSB0";
static SSMApi<ssm::JSData, ssm::JSProperty> *JOY;
static SSMApi<OMcntl> *OMotor;
static SSMApi <config, config_property> *CONF;

int main( int aArgc, char **aArgv )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	SSMApi<ssm::JSData, ssm::JSProperty> joystick( SSM_NAME_JOYSTICK, 0 );
		JOY = &joystick;
	SSMApi<OMcntl> OMcntl( OM_CNTL_SNAME, 0 );
		OMotor = &OMcntl;
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		CONF = &config;
		
	operateJoystick operate;
	operate.openOMController( device_name );	// モータードライバと接続
	
	try {
		setupSSM( );
		setSigInt( );

		operate.printProperty( &joystick.property );
		operate.setConfig( config.property );

		bool update[ 1 ] = { false };
		SSM_tid update_id[ 1 ] = { -1 };
#define INDEX_JOYSTICK			0

		Gprint( "\nStart manualOperation\n" );
		while( !gShutOff ){
			update[ INDEX_JOYSTICK ] = false;
			// get latest data for INDEX_JOYSTICK
			if( update_id[ INDEX_JOYSTICK ] < getTID_top( joystick.getSSMId( ) ) ){
				joystick.readLast(  );
				update[ INDEX_JOYSTICK ] = true; // 最新情報を読み込む
				update_id[ INDEX_JOYSTICK ] = joystick.timeId;
			} else {
				update[ INDEX_JOYSTICK ] = false;
			}

			if( update[ INDEX_JOYSTICK ] ){
				
				operate.operateBotton( &joystick.data );

			} else {
				usleepSSM( dT * 1000 );	// 1ms
			}
			
			// The following MUST be checked 
			bool ret = operate.operateStick( &joystick.data );
			if( ret ){
				OMcntl.data = operate.getOMcntl( );
				OMcntl.write( );
			}
		}
	}
	catch( std::runtime_error const & error ){
		std::cout << error.what() << std::endl;
	}
	catch( ... ){
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}
	
	operate.closeOMController( );
	Terminate( );
	
	return EXIT_SUCCESS;
}
static void Usage( void )
{
	printf( "USAGE\n" );
	printf( "\t START   : Start\n" );
	printf( "\t SELECT  : Free mode\n" );
	printf( "\t PS      : Return Zero Position\n" );
	printf( "\t 丸ボタン: Reset ALARM\n" );
	printf( "\t ☓ボタン: Set Zero Position\n" );
	printf( "\t △ボタン: レバー下ろす\n" );
	printf( "\t □ボタン: レバー上げる\n" );
	printf( "\t 左（上下）: リニアアクチュエータ\n" );
	printf( "\t 右（左右）: 中空アクチュエータ\n" );
	printf( "\n\n" );
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
	Usage( );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	bool flag_device = false;
	int opt, optIndex = 0;
	struct option longOpt[] = {
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

	// get joystick.property
	std::cerr << "\n" << "get joystick.property ... ";
	if( !JOY->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to get joystick.property .");
	else
		std::cerr << "OK.\n";

	// joystick を開く
	std::cerr << "open joystick ... ";
	if( !JOY->open( SSM_READ ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open joystick on ssm.\n" );
	else
		std::cerr << "OK.\n";

	// OMcntl を作成
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
	JOY->release( );
	OMotor->release( );
	CONF->release( );
	endSSM( );
	Gprint( "\nend\n\n" );
}

