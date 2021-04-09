/*
 * Save SSM.property to file
 * 
 * Date   : 2020.02.20
 * Update : 2021.03.21
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

static int gShutOff = 0;
static void setSigInt( void );
static void printShortHelp( const char *programName );
static bool setOption(	int aArgc, char *aArgv[] );
static void setupSSM( void );
static void Terminate( void );

static char path[ STRLEN ] = "../data";
static unsigned int dT = 100;	// [ms]
static bool flag_all_print = false;
static SSMApi <config, config_property> *Conf;

int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;
	
	SSMApi <config, config_property> config( CONFIG_SNAME, 0 );
		Conf = &config;
	commonParam comp;
	comp.initialize( path, flag_all_print, "w" );
	
	try{
		setupSSM( );
		setSigInt( );
		
		Gprint( "\nstart config-viewer\n" );
		comp.setConfProperty( config.property );
		comp.saveProperty( );
		comp.printProperty( );
		
		while( !gShutOff ){
			
			gShutOff = 1;
			
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
	printf( "\t-p | --path   	  PATH     : Path of directory for the save file. (defalut=%s)\n", path );
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

	// get config.property
	std::cerr << "get config.property ... ";
	if( !Conf->getProperty( ) )
		throw std::runtime_error("[\033[1m\033[31mERROR\033[30m\033[0m]:fail to get config.property.");
	else
		std::cerr << "OK.\n";

}
static void Terminate( void )
{
	Conf->release( );
	endSSM( );
	Gprint( "\nend\n" );
}
