/*
 * Date : 2019.05.30
 * Update : 2019.09.03
 * Update : 2019.09.04
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerWP.hpp"
#include <math.h>

void GraphDrawerWP::setParameter( config_property *c, char *path )
{
	setConfig( c );
	create_wp.initialize( path );
}
void GraphDrawerWP::setPose( localizer *data )
{
	if( data->status ){
	
	// ロボットの描写パラメータ
		robot_pos[ _X ] = data->estPose.x;	
		robot_pos[ _Y ] = data->estPose.y;
		robot_pos[ _YAW ] = data->estPose.theta;
	// 推定ポーズの蓄積（localizer）
		pos_data_ring[ pos_data_ring_head ][ _X ] = data->estPose.x;	
		pos_data_ring[ pos_data_ring_head ][ _Y ] = data->estPose.y;
		pos_data_ring_head++;
		if( pos_data_ring_head >= PLOT_DATA_MAX )
			pos_data_ring_head = 0;
		if( pos_data_ring_cnt < PLOT_DATA_MAX )
			pos_data_ring_cnt++;
		else 
			pos_data_ring_cnt = PLOT_DATA_MAX;
	// WPの蓄積
		bool ret = create_wp.setPose( data );
		if( ret ){
			wp_gl wp0 = create_wp.getWP( );
			// WPの蓄積
			wp_data_ring[ wp_data_ring_head ][ _X ] = wp0.x;	
			wp_data_ring[ wp_data_ring_head ][ _Y ] = wp0.y;
			wp_data_ring_head++;
			if( wp_data_ring_head >= PLOT_DATA_MAX )
				wp_data_ring_head = 0;
			if( wp_data_ring_cnt < PLOT_DATA_MAX )
				wp_data_ring_cnt++;
			else 
				wp_data_ring_cnt = PLOT_DATA_MAX;
		}
	}
}
void GraphDrawerWP::drawGraph( void )
{
	fprintf( gp, "p " );

	// 軌跡の描画設定 (localizer)
	if( pos_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'red' t 'localizer', " );
	// WPの描画設定
	if( wp_data_ring_cnt > 0 ){
		fprintf( gp, " '-' pt 5 ps 0.8 lc rgb 'black' t 'WP', " );
	}
	if( wp_data_ring_cnt > 1 ){
		fprintf( gp, " '-' w l lc rgb 'green', " );
	}
	//ロボットの描画設定
	fprintf( gp," '-' w l lc rgb 'black'" );
	fprintf( gp, "\n" );

	//軌跡の描画 (localizer)
	if( pos_data_ring_cnt > 0 ){
		for( int i = 0 ; i < pos_data_ring_cnt ; i++ ){
			fprintf( gp, "%lf %lf\n", pos_data_ring[ i ][ _X ], pos_data_ring[ i ][ _Y ] );
		}
		fprintf( gp, "e\n" );
	}
	
	// WPの描画
	if( wp_data_ring_cnt > 0 ){
		for( int i = 0 ; i < wp_data_ring_cnt ; i++ ){
			fprintf( gp, "%lf %lf\n", wp_data_ring[ i ][ _X ], wp_data_ring[ i ][ _Y ] );
		}
		fprintf( gp, "e\n" );
	}
	if( wp_data_ring_cnt > 1 ){
		for( int i = 1 ; i < wp_data_ring_cnt ; i++ ){
			fprintf( gp, "%lf %lf\n%lf %lf\n\n", wp_data_ring[ i-1 ][ _X ], wp_data_ring[ i-1 ][ _Y ], wp_data_ring[ i ][ _X ], wp_data_ring[ i ][ _Y ] );
		}
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

void GraphDrawerWP::writeSaveFile( void )
{
	create_wp.saveWPFile( );
}
