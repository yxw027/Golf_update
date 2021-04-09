/*
 * Date : 2018.07.09
 * Update : 2019.01.12
 * Update : 2019.02.01
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerHandle.hpp"
#include "utility.hpp"
#include "OMcntl.hpp"
#include <math.h>

using namespace std;

#define _INPUT		0
#define _ANGLE		1
#define _TORQUE		2
#define _ANGVEL		3

#define _TARGET			4
#define _TARGET_VEL		5
#define _TARGET_EX		6

void GraphDrawerHandle::setParameters( int id, bool flag )
{
	flag_save = flag;
	if( flag_save ){
		savefile.openSaveFile( "log_handle.dat" );
		savefile.setMotorID( id );
	}
}

void GraphDrawerHandle::setHandleInfo( OMcntl *data, double t )
{
	if( flag_save ){
		savefile.log2txt( data, t );
	}
	handle_ring[ handle_ring_head ][ _INPUT ] = data->handle.input;
	handle_ring[ handle_ring_head ][ _ANGLE ] = data->handle.ang;
	handle_ring[ handle_ring_head ][ _TORQUE ] = data->handle.torque;
	handle_ring[ handle_ring_head ][ _ANGVEL ] = data->handle.angvel;
	handle_ring[ handle_ring_head ][ _TARGET ] = data->handle.target;
	handle_ring[ handle_ring_head ][ _TARGET_VEL ] = data->handle.target_angvel;
	handle_ring[ handle_ring_head ][ _TARGET_EX ] = data->handle.target_ex;
	
	time[ handle_ring_head ] = t;
	curTime = t;
	handle_ring_head++;
	if( handle_ring_head >= DATA_MAX )
		handle_ring_head = 0;
	if( handle_ring_cnt < DATA_MAX )
		handle_ring_cnt++;

}
void GraphDrawerHandle::drawGraph( void )
{
	if( handle_ring_head > 0 ){
					
		fprintf( gp,"reset\n unset mouse\nset grid\n set key outside below\n" );		

		fprintf( gp, "set title 'Steering Information'\n" );
		fprintf( gp, "set yrange[-540:540]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
		fprintf( gp, "set xtics 5\n" );
		fprintf( gp, "p " );
		fprintf( gp, " '-' pt 5 lc rgb 'green' t 'Torque', " );
		fprintf( gp, " '-' pt 5 lc rgb 'gray' t 'Target'," );
		fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'Input'," );
		fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Angle'" );
		fprintf( gp, "\n" );

		for( int i = 0 ; i < handle_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], handle_ring[ i ][ _TORQUE ] );
		}
		fprintf( gp,"e\n" );
		for( int i = 0 ; i < handle_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], handle_ring[ i ][ _TARGET ] );
		}
		fprintf( gp,"e\n" );
		for( int i = 0 ; i < handle_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], handle_ring[ i ][ _INPUT ] );
		}
		fprintf( gp,"e\n" );
		for( int i = 0 ; i < handle_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], handle_ring[ i ][ _ANGLE ] );
		}
		fprintf( gp,"e\n" );

		fflush( gp );
	}
}


