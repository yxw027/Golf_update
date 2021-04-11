/*
 * GraphDrawerGNSS-f9p.cpp
 * Date : 2019.05.30
 * Update : 2019.08.01
 * Update : 2021.03.16
 * Author : T.Hasegawa
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include "GraphDrawerGNSS-f9p.hpp"
#include "utility.hpp"
#include <math.h>

void GraphDrawerGNSS::setParameters( config_property *c, bool flag )
{
	flag_save = flag;
	setConfig( c );
	
	if( flag_save ){
		savefile.openSaveFile( "log_gnss.dat" );
	}
}

void GraphDrawerGNSS::drawGraph( void )
{
	fprintf( gp, "p " );

	//RTK_FIX軌跡の描画
	if( rtk_fix_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'blue' t 'RTK_FIX', " );
	//RTK_FLOAT軌跡の描画
	if( rtk_float_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'orange' t 'RTK_FLOAT', " );
	//DGPS_FIX軌跡の描画
	if( dgps_fix_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'green' t 'DGPS_FIX', " );
	//SINGLE_FIX軌跡の描画
	if( single_fix_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'red' t 'SINGLE_FIX', " );
	//UNKNOWN軌跡の描画
	if( unknown_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'purple' t 'UNKNOWN', " );
	//ロボットの描画設定
	fprintf( gp," '-' w l lc rgb 'black'" );
	fprintf( gp, "\n" );
				
	//RTK_FIX軌跡
	if( rtk_fix_data_ring_cnt > 0 ){
		for( int i = 0 ; i < rtk_fix_data_ring_cnt ; i++ )
			fprintf( gp, "%lf %lf\n", rtk_fix_data_ring[ i ][ 0 ], rtk_fix_data_ring[ i ][ 1 ] );
		fprintf( gp, "e\n" );
	}
	//RTK_FLOAT軌跡
	if( rtk_float_data_ring_cnt > 0 ){
		for( int i = 0 ; i < rtk_float_data_ring_cnt ; i++ )
			fprintf( gp, "%lf %lf\n", rtk_float_data_ring[ i ][ 0 ], rtk_float_data_ring[ i ][ 1 ] );
		fprintf( gp, "e\n" );
	}
	//DGPS_FIX軌跡
	if( dgps_fix_data_ring_cnt > 0 ){
		for( int i = 0 ; i < dgps_fix_data_ring_cnt ; i++ )
			fprintf( gp, "%lf %lf\n", dgps_fix_data_ring[ i ][ 0 ], dgps_fix_data_ring[ i ][ 1 ] );
		fprintf( gp, "e\n" );
	}
	//SINGLE_FIX軌跡
	if( single_fix_data_ring_cnt > 0 ){
		for( int i = 0 ; i < single_fix_data_ring_cnt ; i++ )
			fprintf( gp, "%lf %lf\n", single_fix_data_ring[ i ][ 0 ], single_fix_data_ring[ i ][ 1 ] );
		fprintf( gp, "e\n" );
	}
	//UNKNOWN軌跡
	if( unknown_data_ring_cnt > 0 ){
		for( int i = 0 ; i < unknown_data_ring_cnt ; i++ )
			fprintf( gp, "%lf %lf\n", unknown_data_ring[ i ][ 0 ], unknown_data_ring[ i ][ 1 ] );
		fprintf( gp, "e\n" );
	}	
	//ロボットの描画
	double odm_tmp[ 3 ];
	odm_tmp[ 0 ] = robot_pos[ _X ];
	odm_tmp[ 1 ] = robot_pos[ _Y ];
	odm_tmp[ 2 ] = robot_pos[ _YAW ];
	fprintf_robot( &robot[ 0 ][ 0 ], 5, odm_tmp );
	fprintf( gp,"e\n" );
	fflush( gp );
}

static rtk_gnss_f9p gnss_old;
static bool flag_first_loop = true;
void GraphDrawerGNSS::setPose( rtk_gnss_f9p *gnss, double t )
{
	if( flag_save ){
		savefile.log2txt( gnss, t );
	}
// ロボットの描写パラメータ
	robot_pos[ _X ] = gnss->enu.x;	//gnss->ecef.x;	
	robot_pos[ _Y ] = gnss->enu.y; //gnss->ecef.y;
	if( flag_first_loop ){
		robot_pos[ _YAW ] = 0;
		flag_first_loop = false;
	} else {
		double dx = gnss->enu.x - gnss_old.enu.x;
		double dy = gnss->enu.y - gnss_old.enu.y;
		robot_pos[ _YAW ] = atan2( dy, dx );
	}
	gnss_old.enu.x = gnss->enu.x;
	gnss_old.enu.y = gnss->enu.y;

	if( gnss->status ){
		if( gnss->posStatus == RTK_FIX ){
			rtk_fix_data_ring[ rtk_fix_data_ring_head ][ 0 ] = gnss->enu.x;	
			rtk_fix_data_ring[ rtk_fix_data_ring_head ][ 1 ] = gnss->enu.y;
			rtk_fix_data_ring_head++;
			if( rtk_fix_data_ring_head >= DATA_MAX )
				rtk_fix_data_ring_head = 0;
			if( rtk_fix_data_ring_cnt < DATA_MAX )
				rtk_fix_data_ring_cnt++;
			else 
				rtk_fix_data_ring_cnt = DATA_MAX;
		} else if( gnss->posStatus == RTK_FLOAT ){
			rtk_float_data_ring[ rtk_float_data_ring_head ][ 0 ] = gnss->enu.x;	
			rtk_float_data_ring[ rtk_float_data_ring_head ][ 1 ] = gnss->enu.y;
			rtk_float_data_ring_head++;
			if( rtk_float_data_ring_head >= DATA_MAX )
				rtk_float_data_ring_head = 0;
			if( rtk_float_data_ring_cnt < DATA_MAX )
				rtk_float_data_ring_cnt++;
			else 
				rtk_float_data_ring_cnt = DATA_MAX;
		} else if( gnss->posStatus == UNKNOWN ){
			unknown_data_ring[ unknown_data_ring_head ][ 0 ] = gnss->enu.x;	
			unknown_data_ring[ unknown_data_ring_head ][ 1 ] = gnss->enu.y;
			unknown_data_ring_head++;
			if( unknown_data_ring_head >= DATA_MAX )
				unknown_data_ring_head = 0;
			if( unknown_data_ring_cnt < DATA_MAX )
				unknown_data_ring_cnt++;
			else 
				unknown_data_ring_cnt = DATA_MAX;
		} else if( gnss->posStatus == SINGLE_FIX ){
			single_fix_data_ring[ single_fix_data_ring_head ][ 0 ] = gnss->enu.x;	
			single_fix_data_ring[ single_fix_data_ring_head ][ 1 ] = gnss->enu.y;
			single_fix_data_ring_head++;
			if( single_fix_data_ring_head >= DATA_MAX )
				single_fix_data_ring_head = 0;
			if( single_fix_data_ring_cnt < DATA_MAX )
				single_fix_data_ring_cnt++;
			else 
				single_fix_data_ring_cnt = DATA_MAX;
		} else if( gnss->posStatus == DGPS_FIX ){
			dgps_fix_data_ring[ dgps_fix_data_ring_head ][ 0 ] = gnss->enu.x;	
			dgps_fix_data_ring[ dgps_fix_data_ring_head ][ 1 ] = gnss->enu.y;
			dgps_fix_data_ring_head++;
			if( dgps_fix_data_ring_head >= DATA_MAX )
				dgps_fix_data_ring_head = 0;
			if( dgps_fix_data_ring_cnt < DATA_MAX )
				dgps_fix_data_ring_cnt++;
			else 
				dgps_fix_data_ring_cnt = DATA_MAX;
		} else {
			fprintf( stderr, "Error? Positioning Status.\n" );
		}
	}
}
