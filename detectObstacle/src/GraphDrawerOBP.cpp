/*
 * Date : 2019.02.01
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "utility.hpp"
#include "GraphDrawerOBP.hpp"

void GraphDrawerOBP_2D::setScan( urg_fs *urg )
{
	scan = *urg;
}
void GraphDrawerOBP_2D::setOBPoint( obp_fs *p )
{
	obp = *p;
}
void GraphDrawerOBP_2D::setParameter( bool laser, bool intensity, config_property *c )
{
	flag_intensity = intensity;
	flag_laser = laser;
	conf = *c;

	for( int i = 0 ; i < NUM_AREA_TYPE ; i++ ){
		printf( "<TYPE #%d>\n", i );
		dec_area[ i ].p1.x = c->dec_area[ i ].p1.x;
		dec_area[ i ].p1.y = c->dec_area[ i ].p1.y;
		dec_area[ i ].p2.x = c->dec_area[ i ].p2.x;
		dec_area[ i ].p2.y = c->dec_area[ i ].p2.y;
		dec_area[ i ].p3.x = c->dec_area[ i ].p3.x;
		dec_area[ i ].p3.y = c->dec_area[ i ].p3.y;
		dec_area[ i ].p4.x = c->dec_area[ i ].p4.x;
		dec_area[ i ].p4.y = c->dec_area[ i ].p4.y;
		stop_area[ i ].p1.x = c->stop_area[ i ].p1.x;
		stop_area[ i ].p1.y = c->stop_area[ i ].p1.y;
		stop_area[ i ].p2.x = c->stop_area[ i ].p2.x;
		stop_area[ i ].p2.y = c->stop_area[ i ].p2.y;
		stop_area[ i ].p3.x = c->stop_area[ i ].p3.x;
		stop_area[ i ].p3.y = c->stop_area[ i ].p3.y;
		stop_area[ i ].p4.x = c->stop_area[ i ].p4.x;
		stop_area[ i ].p4.y = c->stop_area[ i ].p4.y;
		avoid_area[ i ].p1.x = c->avoid_area[ i ].p1.x;
		avoid_area[ i ].p1.y = c->avoid_area[ i ].p1.y;
		avoid_area[ i ].p2.x = c->avoid_area[ i ].p2.x;
		avoid_area[ i ].p2.y = c->avoid_area[ i ].p2.y;
		avoid_area[ i ].p3.x = c->avoid_area[ i ].p3.x;
		avoid_area[ i ].p3.y = c->avoid_area[ i ].p3.y;
		avoid_area[ i ].p4.x = c->avoid_area[ i ].p4.x;
		avoid_area[ i ].p4.y = c->avoid_area[ i ].p4.y;
		printf( "%8.3f # P1 X [m] FOR DECELERATION AREA\n", dec_area[ i ].p1.x );
		printf( "%8.3f #    Y [m] \n", dec_area[ i ].p1.y );
		printf( "%8.3f # P2 X [m] FOR DECELERATION AREA\n", dec_area[ i ].p2.x );
		printf( "%8.3f #    Y [m] \n", dec_area[ i ].p2.y );
		printf( "%8.3f # P3 X [m] FOR DECELERATION AREA\n", dec_area[ i ].p3.x );
		printf( "%8.3f #    Y [m] \n", dec_area[ i ].p3.y );
		printf( "%8.3f # P4 X [m] FOR DECELERATION AREA\n", dec_area[ i ].p4.x );
		printf( "%8.3f #    Y [m] \n\n", dec_area[ i ].p4.y );
		printf( "%8.3f # P1 X [m] FOR EMERGENCY STOP AREA\n", stop_area[ i ].p1.x );
		printf( "%8.3f #    Y [m] \n", stop_area[ i ].p1.y );
		printf( "%8.3f # P2 X [m] FOR EMERGENCY STOP AREA\n", stop_area[ i ].p2.x );
		printf( "%8.3f #    Y [m] \n", stop_area[ i ].p2.y );
		printf( "%8.3f # P3 X [m] FOR EMERGENCY STOP AREA\n", stop_area[ i ].p3.x );
		printf( "%8.3f #    Y [m] \n", stop_area[ i ].p3.y );
		printf( "%8.3f # P4 X [m] FOR EMERGENCY STOP AREA\n", stop_area[ i ].p4.x );
		printf( "%8.3f #    Y [m] \n\n", stop_area[ i ].p4.y );
/*		printf( "%8.3f # P1 X [m] FOR AVOIDANCE AREA\n", avoid_area[ i ].p1.x );
		printf( "%8.3f #    Y [m] \n", avoid_area[ i ].p1.y );
		printf( "%8.3f # P2 X [m] FOR AVOIDANCE AREA\n", avoid_area[ i ].p2.x );
		printf( "%8.3f #    Y [m] \n", avoid_area[ i ].p2.y );
		printf( "%8.3f # P3 X [m] FOR AVOIDANCE AREA\n", avoid_area[ i ].p3.x );
		printf( "%8.3f #    Y [m] \n", avoid_area[ i ].p3.y );
		printf( "%8.3f # P4 X [m] FOR AVOIDANCE AREA\n", avoid_area[ i ].p4.x );
		printf( "%8.3f #    Y [m] \n\n", avoid_area[ i ].p4.y );
*/
	}
}
void GraphDrawerOBP_2D::drawGraph( void )
{
	fprintf( gp, "p " );

	if( flag_laser )
		fprintf( gp, " '-' w l lc rgb 'green', " );

	if( flag_intensity )
		fprintf( gp, "'-' pt 5 ps 0.2 pal, " );
	else
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'red', " );

	//障害物の最短点
	if( obp.status == STOP ){
		fprintf( gp, "'-' pt 7 lc rgb 'black', " );
	} else if( obp.status == DECELERATION ){ 
		fprintf( gp, "'-' pt 7 lc rgb 'gray', " );
	} else { }

	// 衝突回避領域の描画
//	fprintf( gp, " '-' w l lc rgb 'purple', " );
	fprintf( gp, " '-' w l lc rgb 'purple', " );
	fprintf( gp, " '-' w l lc rgb 'purple'" );
	fprintf( gp, "\n" );

	if( flag_laser ){
		for( int i = 0 ; i < scan.size ; i++ ){
			fprintf( gp, "0 0\n%lf %lf\n\n", scan.length[ i ]*cos( scan.angle[ i ] ), scan.length[ i ]*sin( scan.angle[ i ] ) );
		}
		fprintf( gp, "e\n" );
	}

	for( int i = 0 ; i < scan.size ; i++ ){
		fprintf( gp, "%lf %lf %d\n", scan.length[ i ]*cos( scan.angle[ i ] ), scan.length[ i ]*sin( scan.angle[ i ] ), scan.intensity[ i ] );
	}
	fprintf( gp, "e\n" );
				
	//障害物の最短点
	if( obp.status == STOP || obp.status == DECELERATION ){
		fprintf( gp, "%lf %lf\n", obp.pos[ 0 ], obp.pos[ 1 ] );
		fprintf( gp, "e\n" );
	}

	// 衝突回避領域の描画
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.dec_area[ area_type ].p1.x, conf.dec_area[ area_type ].p1.y, conf.dec_area[ area_type ].p2.x, conf.dec_area[ area_type ].p2.y );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.dec_area[ area_type ].p2.x, conf.dec_area[ area_type ].p2.y, conf.dec_area[ area_type ].p3.x, conf.dec_area[ area_type ].p3.y );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.dec_area[ area_type ].p3.x, conf.dec_area[ area_type ].p3.y, conf.dec_area[ area_type ].p4.x, conf.dec_area[ area_type ].p4.y );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.dec_area[ area_type ].p4.x, conf.dec_area[ area_type ].p4.y, conf.dec_area[ area_type ].p1.x, conf.dec_area[ area_type ].p1.y );
	fprintf( gp, "e\n" );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.stop_area[ area_type ].p1.x, conf.stop_area[ area_type ].p1.y, conf.stop_area[ area_type ].p2.x, conf.stop_area[ area_type ].p2.y );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.stop_area[ area_type ].p2.x, conf.stop_area[ area_type ].p2.y, conf.stop_area[ area_type ].p3.x, conf.stop_area[ area_type ].p3.y );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.stop_area[ area_type ].p3.x, conf.stop_area[ area_type ].p3.y, conf.stop_area[ area_type ].p4.x, conf.stop_area[ area_type ].p4.y );
	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.stop_area[ area_type ].p4.x, conf.stop_area[ area_type ].p4.y, conf.stop_area[ area_type ].p1.x, conf.stop_area[ area_type ].p1.y );
	fprintf( gp, "e\n" );
//	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.avoid_area[ area_type ].p1.x, conf.avoid_area[ area_type ].p1.y, conf.avoid_area[ area_type ].p2.x, conf.avoid_area[ area_type ].p2.y );
//	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.avoid_area[ area_type ].p2.x, conf.avoid_area[ area_type ].p2.y, conf.avoid_area[ area_type ].p3.x, conf.avoid_area[ area_type ].p3.y );
//	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.avoid_area[ area_type ].p3.x, conf.avoid_area[ area_type ].p3.y, conf.avoid_area[ area_type ].p4.x, conf.avoid_area[ area_type ].p4.y );
//	fprintf( gp, "%lf %lf\n%lf %lf\n\n", conf.avoid_area[ area_type ].p4.x, conf.avoid_area[ area_type ].p4.y, conf.avoid_area[ area_type ].p1.x, conf.avoid_area[ area_type ].p1.y );
//	fprintf( gp, "e\n" );
				
	fflush( gp );
}
void GraphDrawerOBP_3D::drawGraph( void )
{
	fprintf( gp, "set zrange [-1.0:1.5]\n" );
	fprintf( gp, "splot " );

	if( flag_intensity )
		fprintf( gp, "'-' pt 5 ps 0.2 pal, " );
	else
		fprintf( gp, "'-' pt 5 ps 0.2 fc rgb 'red', " );

	//障害物の最短点
	if( obp.status == STOP ){
		fprintf( gp, "'-' pt 7 lc rgb 'black', " );
	} 
	// 衝突回避領域の描画
	fprintf( gp, " '-' w l lc rgb 'purple'" );
	fprintf( gp, "\n" );

 
	double height = ( double )conf.urg3d.offset[ _Z ] / 1000.0;
	double theta = DEG2RAD( conf.urg3d.rot[ _PITCH ] );
	for( int i = 1 ; i < scan.size ; i++ ){
		// 座標変換
		double xx = scan.length[ i ] * cos( scan.angle[ i ] );
		double yy = scan.length[ i ] * sin( scan.angle[ i ] );
		double tx = xx * cos( theta );
		double ty = yy;
		double tz = -1.0 * xx * sin( theta ) + height;
		
		fprintf( gp, "%lf %lf %lf %d\n", tx, ty, tz, scan.intensity[ i ] );
	}
	fprintf( gp, "e\n" );
			
	//障害物の最短点
	if( obp.status == STOP ){
		fprintf( gp, "%lf %lf %lf\n", obp.pos[ 0 ], obp.pos[ 1 ], obp.pos[ 2 ] );
		fprintf( gp, "e\n" );
	}

	// 衝突回避領域の描画
	fprintf( gp, "%lf %lf 0\n%lf %lf 0\n\n", conf.stop_area[ area_type ].p1.x, conf.stop_area[ area_type ].p1.y, conf.stop_area[ area_type ].p2.x, conf.stop_area[ area_type ].p2.y );
	fprintf( gp, "%lf %lf 0\n%lf %lf 0\n\n", conf.stop_area[ area_type ].p2.x, conf.stop_area[ area_type ].p2.y, conf.stop_area[ area_type ].p3.x, conf.stop_area[ area_type ].p3.y );
	fprintf( gp, "%lf %lf 0\n%lf %lf 0\n\n", conf.stop_area[ area_type ].p3.x, conf.stop_area[ area_type ].p3.y, conf.stop_area[ area_type ].p4.x, conf.stop_area[ area_type ].p4.y );
	fprintf( gp, "%lf %lf 0\n%lf %lf 0\n\n", conf.stop_area[ area_type ].p4.x, conf.stop_area[ area_type ].p4.y, conf.stop_area[ area_type ].p1.x, conf.stop_area[ area_type ].p1.y );
	fprintf( gp, "e\n" );
		
	fflush( gp );
}
