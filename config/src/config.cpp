/*
 * Set configuration parameters to SSM.property
 * 
 * Date    : 2018.12.11
 * Updated : 2019.03.31
 * Updated : 2019.08.08 
 * Updated : 2021.03.21
 * Author : T.Hasegawa
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <stdexcept>
#include <errno.h>
#include <fcntl.h>

#include <sys/stat.h>
#include <sys/termios.h>
#include <time.h>

#include <ssm.hpp>
#include "config.hpp"
#include "commonParam.hpp"
#include "localizer.hpp"
#include "wp.hpp"
#include "detectObstacle.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static char path[ STRLEN ] = "../param";
static unsigned int dT = 100;	// [ms]
static bool flag_all_print = false;
static SSMApi <config, config_property> *Conf;
static SSMApi <localizer> *LOCAL;
static SSMApi <wp_gl> *WP;
static SSMApi <obp_fs> *OBPoint;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		Conf = &config;
	SSMApi <localizer> localizer( LOCALIZER_SNAME, 0 );
		LOCAL = &localizer;
	SSMApi <wp_gl> wp_gl( WP_SNAME, 0 );
		WP = &wp_gl;
	SSMApi<obp_fs> obp_fs( OBP_SNAME, 0 );
		OBPoint = &obp_fs;
	
	commonParam comp;
	comp.initialize( path, flag_all_print, "r" );
	config.property = comp.getConfProperty( );
				
	try{
		setupSSM( );
		setSigInt( );
		
		Gprint( "\nstart config\n" );
		comp.printProperty( );
		
		while( !gShutOff ){
			
			gShutOff = 1;
			
			for( int i = 0 ; i < 2 ; i++ ){
				// config
				config.data.dummy = 999;
				config.write( );
				// localizer
				localizer.data.status = false;
				localizer.data.estPose = config.property.init_pose;
				localizer.write( );
				// wp_gl
				wp_gl.data.id = 0;
				wp_gl.data.x = config.property.init_pose.x;
				wp_gl.data.y = config.property.init_pose.y;
				wp_gl.data.theta = config.property.init_pose.theta;
				wp_gl.data.v = 0;
				wp_gl.data.flag_cut = 0;
				wp_gl.data.area_type = 0;
				wp_gl.write( );
				// detectObstacle
				obp_fs.data.status = TRAVELING;
				obp_fs.write( );
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
	fprintf( stderr, "\t$ %s -p %s\n", programName, path );
	printf( "OPTION\n" );
	printf( "\t-p | --path   	  PATH     : Path of directory for the parameter file. (defalut=%s)\n", path );
	printf( "\t-a | --print   	  PRINT    : Print all parameters. (defalut=%d, all:1)\n", flag_all_print );
	printf( "\n" );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "path", 1, 0, 'p' },
		{ "print", 1, 0, 'a' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};
	while( ( opt = getopt_long( aArgc, aArgv, "p:ah", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
		case 'p':
			strcpy( path, optarg );
			break;
		case 'a':
			flag_all_print = true;
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
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to initialize ssm." );
	else
		std::cerr << "OK.\n";

	// configを作成
	std::cerr << "create config ... ";
	if( !Conf->create( 1, ( double )dT/1000.0 ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create config on ssm.\n" );
	else
		std::cerr << "OK.\n";
	
	// set configuration
	std::cerr << "set config.property ... ";
	if( !Conf->setProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to set property for configuration.");
	else
		std::cerr << "OK.\n";
		
	// Localizer作成
	std::cerr << "create localizer ... ";
	if( !LOCAL->create( 1, ( double )dT/1000.0 ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create localizer on ssm.\n");
	else
		std::cerr << "OK.\n";
	
	// wp_glを生成
	std::cerr << "create wp_gl ... ";
	if( !WP->create( 5, ( double )dT/1000.0 ) )
		throw std::runtime_error( "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create wp_gl on ssm.\n" );
	else
		std::cerr << "OK.\n";

	// obp_fs を作成
	std::cerr << "create obp_fs ... ";
	if( !OBPoint->create( 1, ( double )dT/1000.0 ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to create obp_fs on ssm.\n");
	else
		std::cerr << "OK.\n";
		
}
static void Terminate( void )
{
	Conf->release( );
	LOCAL->release( );
	WP->release( );
	OBPoint->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
