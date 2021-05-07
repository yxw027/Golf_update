/*
 * Date : 2019.05.30
 * Update : 2019.09.03
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerPose.hpp"
#include <math.h>

void GraphDrawerPose::setParameters( config_property *c, bool flag )
{
	flag_save = flag;
	setConfig( c );
	if( flag_save ){
		savefile.openSaveFile( "log_localizer.dat" );
	}
}

void GraphDrawerPose::drawGraph( void )
{
	fprintf( gp, "p " );

	// 軌跡の描画設定 (localizer)
	if( pos_data_ring_cnt > 0 )
		fprintf( gp, "'-' pt 5 ps 0.5 lc rgb 'red' t 'localizer', " );
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
	
	//ロボットの描画
	double odm_tmp[ 3 ];
	odm_tmp[ 0 ] = robot_pos[ _X ];
	odm_tmp[ 1 ] = robot_pos[ _Y ];
	odm_tmp[ 2 ] = robot_pos[ _YAW ];
	fprintf_robot( &robot[ 0 ][ 0 ], 5, odm_tmp );
	fprintf( gp,"e\n" );
	fflush( gp );
}
void GraphDrawerPose::setPose( localizer *data, double t )
{
	if( flag_save ){
		savefile.log2txt( data, t );
	}

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
}
