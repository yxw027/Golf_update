/*
 * Date : 2020.03.12
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerVelocity.hpp"
#include "utility.hpp"
#include "localizer.hpp"
#include <math.h>

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) (x)
#else
#define DEBUG_PRINT(x)
#endif

using namespace std;

void GraphDrawerVel::setParameters( bool flag )
{
	flag_save = flag;
	if( flag_save ){
		savefile.openSaveFile( "log_localizer.dat" );
	}
}
static bool flag_first_loop = true;
static double start_time = 0;
static double odm_x_old = 0, odm_y_old = 0;
void GraphDrawerVel::setVel( localizer *odm, double t )
{
	if( flag_save ){
		savefile.log2txt( odm, t );
	}
	
	if( flag_first_loop ){
		start_time = t;
		flag_first_loop = false;
		
		odm_x_old = odm->estPose.x;
		odm_y_old = odm->estPose.y;
	}
	vel_ring[ vel_ring_head ] = odm->estPose.v;
	time[ vel_ring_head ] = t - start_time;
	curTime = t - start_time;

	// 確認用表示
	DEBUG_PRINT( printPose( odm, curTime - time[ vel_ring_head-1 ]) );
	
	vel_ring_head++;
	if( vel_ring_head >= DATA_MAX )
		vel_ring_head = 0;
	if( vel_ring_cnt < DATA_MAX )
		vel_ring_cnt++;

}
void GraphDrawerVel::printPose( localizer *odm, double dt )
{
	double dx = odm->estPose.x - odm_x_old;
	double dy = odm->estPose.y - odm_y_old;
	double diff = sqrt( dx*dx + dy*dy );
	double tv = diff / dt;
	double dv_x = dx / dt;
	double dv_y = dy / dt;
	double dv = sqrt( dv_x*dv_x + dv_y*dv_y );
	printf( "x=%f, y=%f, v=%f(%f:%f), dx=%f, dy=%f, diff=%f, dt=%f\n", odm->estPose.x, odm->estPose.y, odm->estPose.v, tv, dv, dx, dy, diff, dt );
	odm_x_old = odm->estPose.x;
	odm_y_old = odm->estPose.y;
}
void GraphDrawerVel::drawGraph( void )
{
	if( vel_ring_cnt > 0 ){
					
		fprintf( gp,"reset\n unset mouse\nset grid\n set key outside below\n" );		

		fprintf( gp, "set title 'Velocity'\n" );
		fprintf( gp, "set yrange[0:2.5]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
		fprintf( gp, "set xtics 5\n" );
		fprintf( gp, "p " );
		if( vel_ring_cnt > 1 ){
			fprintf( gp, " '-' w l lc rgb 'green', " );
		}
		fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Velocity'" );
		fprintf( gp, "\n" );

		if( vel_ring_cnt > 1 ){
			for( int i = 1 ; i < vel_ring_cnt ; i++ ){
				fprintf( gp, "%lf %lf\n%lf %lf\n\n", time[ i-1 ], vel_ring[ i-1 ], time[ i ], vel_ring[ i ] );
			}
			fprintf( gp, "e\n" );
		}
		for( int i = 0 ; i < vel_ring_cnt ; i++ ){
			fprintf( gp,"%lf %lf\n", time[ i ], vel_ring[ i ] );
		}
		fprintf( gp,"e\n" );
	
		fflush( gp );
	}
}
