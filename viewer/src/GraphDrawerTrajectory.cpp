/*
 * Date : 2018.07.09
 * Update : 2021.04.04
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerTrajectory.hpp"
#include "utility.hpp"
#include <math.h>

using namespace std;
// *********************************************************************
// ********************* SSMからWPを受け取り表示 ***********************
// *********************************************************************
void GraphDrawerTrajectory::setParameters( config_property *c, bool lflag, bool cflag )
{
	conf = *c;
	setConfig( c );
	flag_save_localizer = lflag;
	flag_save_control = cflag;
}

void GraphDrawerTrajectory::setWP( wp_gl *wp )
{
	wp_data_ring[ wp_data_ring_head ][ _X ] = wp->x;
	wp_data_ring[ wp_data_ring_head ][ _Y ] = wp->y;
	wp_id_ring[ wp_data_ring_head ] = wp->id;
	wp_data_ring_head++;
	if( wp_data_ring_head >= WP_DATA_MAX )
		wp_data_ring_head = 0;
	if( wp_data_ring_cnt < WP_DATA_MAX )
		wp_data_ring_cnt++;
	else 
		wp_data_ring_cnt = WP_DATA_MAX;
}

void GraphDrawerTrajectory::setPose( localizer *est, double t )
{
	if( flag_save_localizer ){
		savefile_localizer.log2txt( est, t );
	}
	
	robot_pos[ _X ] = est->estPose.x;	
	robot_pos[ _Y ] = est->estPose.y;
	robot_pos[ _YAW ] = est->estPose.theta;

	pos_data_ring[ pos_data_ring_head ][ _X ] = est->estPose.x;	
	pos_data_ring[ pos_data_ring_head ][ _Y ] = est->estPose.y;

	pos_data_ring_head++;
	if( pos_data_ring_head >= PLOT_DATA_MAX )
		pos_data_ring_head = 0;
	if( pos_data_ring_cnt < PLOT_DATA_MAX )
		pos_data_ring_cnt++;
	else 
		pos_data_ring_cnt = PLOT_DATA_MAX;
}
void GraphDrawerTrajectory::setControlInfo( control *data, double t )
{
	if( flag_save_control ){
		savefile_control.log2txt( data, t );
	}
}

void GraphDrawerTrajectory::drawGraph( void )
{
	// Draw ID for WP
	if( wp_data_ring_cnt > 0 ){
		for( int i = 1 ; i < wp_data_ring_cnt ; i++ ){
			fprintf( gp, "set label \"\%2d\" at %lf+0.1,%lf+0.1\n", wp_id_ring[ i ], wp_data_ring[ i ][ _X ], wp_data_ring[ i ][ _Y ] );
		}
	}
	
	fprintf( gp, "p " );

	// 軌跡の描画設定 (localizer)
	if( pos_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.5 lc rgb 'red' t 'localizer', " );
//		fprintf( gp, "'-' pt 5 ps 0.2 lc rgb 'red' t 'localizer', " );
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
// ***************************************************************************************************
// ********************* SSMから先読みWPを受け取り表示 && 本来のWPも同時に表示 ***********************
// ***************************************************************************************************
void GraphDrawerTrajectory_Proactive::setParameters( config_property *c, bool lflag, bool cflag )
{
	GraphDrawerTrajectory::setParameters( c, lflag, cflag );

	wp_mgr.initilize( conf, 1, NAVI );
	// スタート地点のWPを登録
	wp_gl wp = wp_mgr.getWPcurrent( );
	wp_data_ring[ wp_data_ring_head ][ _X ] = wp.x;
	wp_data_ring[ wp_data_ring_head ][ _Y ] = wp.y;
	wp_id_ring[ wp_data_ring_head ] = wp.id;
	wp_data_ring_head++;
	if( wp_data_ring_head >= WP_DATA_MAX )
		wp_data_ring_head = 0;
	if( wp_data_ring_cnt < WP_DATA_MAX )
		wp_data_ring_cnt++;
	else 
		wp_data_ring_cnt = WP_DATA_MAX;
}

void GraphDrawerTrajectory_Proactive::setWP( wp_gl *wp )
{
	// 先読みWPを登録
	wp_proactive_data_ring[ wp_proactive_data_ring_head ][ _X ] = wp->x;
	wp_proactive_data_ring[ wp_proactive_data_ring_head ][ _Y ] = wp->y;
//	wp_proactive_id_ring[ wp_proactive_data_ring_head ] = wp->id;
	wp_proactive_data_ring_head++;
	if( wp_proactive_data_ring_head >= WP_DATA_MAX )
		wp_proactive_data_ring_head = 0;
	if( wp_proactive_data_ring_cnt < WP_DATA_MAX )
		wp_proactive_data_ring_cnt++;
	else 
		wp_proactive_data_ring_cnt = WP_DATA_MAX;
}
void GraphDrawerTrajectory_Proactive::setPose( localizer *est, double t )
{
	GraphDrawerTrajectory::setPose( est, t );

//	wp_mgr.printWPcurrent( );	// 確認用

//	wp_mgr.getWP( est );
//	wp_mgr.calcWPOffset( );
	// 到達判定後にWPを登録
	if( wp_mgr.chkOverLine( est ) ){
		wp_mgr.CountUpWP( );
		wp_gl wp = wp_mgr.getWPcurrent( );

//		printf("WP%d\n",wp.id);		// 確認用
		wp_data_ring[ wp_data_ring_head ][ _X ] = wp.x;
		wp_data_ring[ wp_data_ring_head ][ _Y ] = wp.y;
		wp_id_ring[ wp_data_ring_head ] = wp.id;
		wp_data_ring_head++;
		if( wp_data_ring_head >= WP_DATA_MAX )
			wp_data_ring_head = 0;
		if( wp_data_ring_cnt < WP_DATA_MAX )
			wp_data_ring_cnt++;
		else 
			wp_data_ring_cnt = WP_DATA_MAX;
	}

}
void GraphDrawerTrajectory_Proactive::drawGraph( void )
{
	// Draw ID for WP
	if( wp_data_ring_cnt > 0 ){
		for( int i = 1 ; i < wp_data_ring_cnt ; i++ ){
			fprintf( gp, "set label \"\%2d\" at %lf+0.1,%lf+0.1\n", wp_id_ring[ i ], wp_data_ring[ i ][ _X ], wp_data_ring[ i ][ _Y ] );
		}
	}
	
	fprintf( gp, "p " );

	// 軌跡の描画設定 (localizer)
	if( pos_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.5 lc rgb 'red' t 'localizer', " );
	// WPの描画設定
	if( wp_data_ring_cnt > 0 ){
		fprintf( gp, " '-' pt 5 ps 0.8 lc rgb 'black' t 'WP', " );
	}
	if( wp_data_ring_cnt > 1 ){
		fprintf( gp, " '-' w l lc rgb 'green', " );
	}
	// 先読みWPの描画設定
	if( wp_proactive_data_ring_cnt > 0 ){
		fprintf( gp, " '-' pt 5 ps 0.8 lc rgb 'red' t 'WP Proactive', " );
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

	// 先読みWPの描画
	if( wp_proactive_data_ring_cnt > 0 ){
		for( int i = 0 ; i < wp_proactive_data_ring_cnt ; i++ ){
			fprintf( gp, "%lf %lf\n", wp_proactive_data_ring[ i ][ _X ], wp_proactive_data_ring[ i ][ _Y ] );
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
