/*
 * Date : 2018.07.09
 * Update : 2019.01.12
 * Update : 2019.02.01
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerAccel.hpp"
#include "utility.hpp"
#include "OMcntl.hpp"
#include <math.h>

using namespace std;

#define _INPUT		0
#define _STROKE		1
#define _TORQUE		2

void GraphDrawerAccel::setParameters( int id, bool flag )
{
	flag_save = flag;
	if( flag_save ){
		savefile.openSaveFile( "log_accel.dat" );
		savefile.setMotorID( id );
	}
}
void GraphDrawerAccel::setAccelInfo( OMcntl *data, double t )
{
	if( flag_save ){
		savefile.log2txt( data, t );
	}
	accel_ring[ accel_ring_head ][ _INPUT ] = data->accel.input;
	accel_ring[ accel_ring_head ][ _STROKE ] = data->accel.pos;
	accel_ring[ accel_ring_head ][ _TORQUE ] = data->accel.torque;
	time[ accel_ring_head ] = t;
	curTime = t;
	accel_ring_head++;
	if( accel_ring_head >= DATA_MAX )
		accel_ring_head = 0;
	if( accel_ring_cnt < DATA_MAX )
		accel_ring_cnt++;

}
void GraphDrawerAccel::drawGraph( void )
{
	if( accel_ring_head > 0 ){
					
		fprintf( gp,"reset\n unset mouse\nset grid\n set key outside below\n" );		

		fprintf( gp, "set title 'Accel Pedal Information'\n" );
		fprintf( gp, "set yrange[0:10]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
		fprintf( gp, "set xtics 5\n" );
		fprintf( gp, "p " );
		fprintf( gp, " '-' pt 5 lc rgb 'green' t 'Torque'," );
		fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'Input'," );
		fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Stroke'" );
		fprintf( gp, "\n" );

		for( int i = 0 ; i < accel_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], accel_ring[ i ][ _TORQUE ] );
		}
		fprintf( gp,"e\n" );
		for( int i = 0 ; i < accel_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], accel_ring[ i ][ _INPUT ] );
		}
		fprintf( gp,"e\n" );
		for( int i = 0 ; i < accel_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], accel_ring[ i ][ _STROKE ] );
		}
		fprintf( gp,"e\n" );
		fflush( gp );
	}
}

