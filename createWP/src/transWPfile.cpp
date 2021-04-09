/*
 * GNSSの設定を間違えたWPファイルの構成を引き継いで、正しい設定のWPファイルを修正する
 * 
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
#include "wp.hpp"
#include "utility.hpp"

static int gShutOff = 0;
static void setSigInt( void );
static bool setOption(	int aArgc, char *aArgv[] );
static void Terminate( void );

static unsigned int wait_time = 100;	// 100ms
	
int main( int aArgc, char *aArgv[ ] )
{
	if( !setOption( aArgc, aArgv ) ) return EXIT_FAILURE;

	try {
		setSigInt( );

		Gprint( "Start transWPfile\n" );
		// オリジナル読込ファイルをオープン
		char tmp[ STRLEN ];
		sprintf( tmp, "../data/WP_orig.dat" );
		printf( "Reading File = %s\n", tmp );	// 確認用
		FILE *fp_inp_orig = isValidFile( tmp, "r", "dat" );
		unsigned int wp_num_orig = atoi( getWord( fp_inp_orig ) );
//		printf( "wp_num_orig=%d\n", wp_num_orig );
		wp_gl *wp_orig;
		if( ( wp_orig = new wp_gl[ wp_num_orig ] ) == NULL ){
			std::cerr << "Cannot allocate WP_orig variables." << std::endl;
			exit( EXIT_FAILURE );
		}
		for( int i = 0 ; i < wp_num_orig ; i++ ){
			wp_orig[ i ].id = atoi( getWord( fp_inp_orig ) );
			wp_orig[ i ].x = atof( getWord( fp_inp_orig ) );
			wp_orig[ i ].y = atof( getWord( fp_inp_orig ) );
			wp_orig[ i ].v = atof( getWord( fp_inp_orig ) );
			wp_orig[ i ].flag_cut = atoi( getWord( fp_inp_orig ) );
			wp_orig[ i ].area_type = atoi( getWord( fp_inp_orig ) );
//			printf( "[%d]%d, ", i, wp_orig[ i ].id );
		}
		fclose( fp_inp_orig );
		
		// 新しい読込ファイルをオープン
		sprintf( tmp, "../data/WP_new.dat" );
		printf( "Reading File = %s\n", tmp );	// 確認用
		FILE *fp_inp_new = isValidFile( tmp, "r", "dat" );
		unsigned int wp_num_new = atoi( getWord( fp_inp_new ) );
//		printf( "wp_num_new=%d\n", wp_num_new );
		wp_gl *wp_new;
		if( ( wp_new = new wp_gl[ wp_num_new ] ) == NULL ){
			std::cerr << "Cannot allocate WP_new variables." << std::endl;
			exit( EXIT_FAILURE );
		}
		for( int i = 0 ; i < wp_num_new ; i++ ){
			wp_new[ i ].id = atoi( getWord( fp_inp_new ) );
			wp_new[ i ].x = atof( getWord( fp_inp_new ) );
			wp_new[ i ].y = atof( getWord( fp_inp_new ) );
			wp_new[ i ].v = atof( getWord( fp_inp_new ) );
			wp_new[ i ].flag_cut = atoi( getWord( fp_inp_new ) );
			wp_new[ i ].area_type = atoi( getWord( fp_inp_new ) );
//			printf( "[%d]%d, ", i, wp_new[ i ].id );
		}
		fclose( fp_inp_new );
		
		// 書込ファイルをオープン
		sprintf( tmp, "../data/WP_revise.dat" );
		printf( "Save File = %s\n", tmp );	// 確認用
		FILE *fp_outp = isValidFile( tmp, "w", "dat" );
		fprintf( fp_outp, "#    Num. of WP\n" );
		fprintf( fp_outp, "    %d\n", wp_num_orig );
		fprintf( fp_outp, "#    ID    WP_x[m]         WP_y[m]          Velocity[m/s]		CUT_Flag[1:down, 2:up]		area_type\n" );
		
		unsigned int wp_cnt = 0;
		while( !gShutOff ){

			for( int i = 0 ; i < wp_num_new ; i++ ){
				if( wp_orig[ wp_cnt ].id == wp_new[ i ].id ){
					fprintf( fp_outp, "%6d %15.4f %15.4f %15.4f	%15d %25d\n", 
							wp_new[ i ].id, wp_new[ i ].x, wp_new[ i ].y, wp_orig[ wp_cnt ].v, wp_orig[ wp_cnt ].flag_cut, wp_new[ i ].area_type );
					wp_cnt++;
					if( wp_cnt >= wp_num_orig ) break;
				}
			}

//			usleepSSM( wait_time * 1000 );
			gShutOff = 1;
		}
		fclose( fp_outp );
	}
	catch (std::runtime_error const & error){
		std::cout << error.what() << std::endl;
	}
	catch (...){
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}
//	drawer.closeSaveFile( );
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
//	fprintf( stderr, "\t$ %s -p ../data\n", programName );
	fputs( "OPTION\n", stderr );
//	printf( "\t-p | --path   	  PATH     : Path of directory for output (default=%s)\n", path );
	printf( "\t-t | --s_time      TIME     : Wait time (defautl=%dms)\n", wait_time );
}
static bool setOption(	int aArgc, char *aArgv[] )
{
	int opt, optIndex = 0;
	struct option longOpt[ ] = {
		{ "s_time", 1, 0, 't' },
//		{ "path", 1, 0, 'p' },
		{ "help", 0, 0, 'h' },
		{ 0, 0, 0, 0 }
	};

	while( ( opt = getopt_long( aArgc, aArgv, "p:t:h", longOpt, &optIndex ) ) != -1 ){
		switch ( opt ){
/*		case 'p':
			strcpy( path, optarg );
			break;
*/		case 't':
			wait_time = atoi( optarg );    
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
static void Terminate( void )
{
	Gprint( "end\n" );
}
