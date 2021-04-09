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

/*
static unsigned long counter = 1;
static bool flag_first_loop = true;
static double pos_prev[ 3 ] = { 0 };
static bool flag_second_loop = true;
static double init_pose[ 3 ] = { 0 };
static double vel_d = 0.0; // [m/s]
static unsigned int cflag = 0;
static unsigned int atype = 0;
static unsigned int gid = 0;
static bool flag_vehicleDirection_old;
static double WP_theta_old;
void GraphDrawerWP::saveWPFile( localizer *data, double time )
{
	if( data->status ){
		
		if( flag_first_loop ){
		
#ifdef NUM_GAIN_TYPE
			fprintf( fp, "%6lu %15.4f %15.4f %15.4f	%15d %25d %12d\n", counter, data->estPose.x, data->estPose.y, vel_d, cflag, atype, gid );
#else
			fprintf( fp, "%6lu %15.4f %15.4f %15.4f	%15d %25d\n", counter, data->estPose.x, data->estPose.y, vel_d, cflag, atype );
#endif
			counter++;
			pos_prev[ _X ] = data->estPose.x;
			pos_prev[ _Y ] = data->estPose.y;
			pos_prev[ _YAW ] = data->estPose.theta;
			flag_first_loop = false;
			flag_vehicleDirection_old = data->flag_slip;
			WP_theta_old = data->estPose.theta;

			init_pose[ _X ] = data->estPose.x;
			init_pose[ _Y ] = data->estPose.y;
			
			// WPの蓄積
			wp_data_ring[ wp_data_ring_head ][ _X ] = data->estPose.x;	
			wp_data_ring[ wp_data_ring_head ][ _Y ] = data->estPose.y;
			wp_data_ring_head++;
			if( wp_data_ring_head >= PLOT_DATA_MAX )
				wp_data_ring_head = 0;
			if( wp_data_ring_cnt < PLOT_DATA_MAX )
				wp_data_ring_cnt++;
			else 
				wp_data_ring_cnt = PLOT_DATA_MAX;
		
		} else {
		
			double dx = data->estPose.x - pos_prev[ _X ];
			double dy = data->estPose.y - pos_prev[ _Y ];
			double distance = sqrt( dx * dx + dy * dy );
			double dtheta = trans_q( data->estPose.theta - pos_prev[ _YAW ] );
//			fprintf(stderr,"d=%lf, th=%lf\n",distance, dTheta*180/M_PI );

			if( ( distance > UPDATE_DIST ) || ( fabs( dtheta ) > UPDATE_THETA ) || ( flag_vehicleDirection_old != data->flag_slip ) ){
				
				if( flag_vehicleDirection_old == data->flag_slip ){ // data->flag_slip(true:前進, false:後退)
					if( data->flag_slip ){
						vel_d = data->gnss_vel[ _V ];
					} else {
						vel_d = -1.0 * data->gnss_vel[ _V ];
					}
				} else { // 前後退の切替え時
					double WP_theta = atan2( data->estPose.y - pos_prev[ _Y ], data->estPose.x - pos_prev[ _X ] );
//					printf("%f - %f = %f\n", WP_theta, WP_theta_old, fabs( WP_theta - WP_theta_old ) );	// 確認用
					if( fabs( WP_theta - WP_theta_old ) < M_PI/4.0 ){
						if( flag_vehicleDirection_old ){
							vel_d = data->gnss_vel[ _V ];
						} else {
							vel_d = -1.0 * data->gnss_vel[ _V ];
						}
					} else {
						if( data->flag_slip ){
							vel_d = data->gnss_vel[ _V ];
						} else {
							vel_d = -1.0 * data->gnss_vel[ _V ];
						}
					}
				}
				vel_d = ( ( double )( int )( vel_d * 10 ) ) / 10.0;	// 小数点以下１桁にする
#ifdef NUM_GAIN_TYPE
				fprintf( fp, "%6lu %15.4f %15.4f %15.4f	%15d %25d %12d\n", counter, data->estPose.x, data->estPose.y, vel_d, cflag, atype, gid );
#else
				fprintf( fp, "%6lu %15.4f %15.4f %15.4f	%15d %25d\n", counter, data->estPose.x, data->estPose.y, vel_d, cflag, atype );
#endif
				counter++;
			
				WP_theta_old = atan2( data->estPose.y - pos_prev[ _Y ], data->estPose.x - pos_prev[ _X ] );
				pos_prev[ _X ] = data->estPose.x;
				pos_prev[ _Y ] = data->estPose.y;
				pos_prev[ _YAW ] = data->estPose.theta;
				// WPファイル確認用
				fprintf( fp4chk, "%f %f %f %f %f\n", time, data->estPose.x, data->estPose.y, WP_theta_old, vel_d );
								
				if( flag_second_loop ){
					flag_second_loop = false;
					init_pose[ _YAW ] = atan2( dy, dx );
				}
				
				// WPの蓄積
				wp_data_ring[ wp_data_ring_head ][ _X ] = data->estPose.x;	
				wp_data_ring[ wp_data_ring_head ][ _Y ] = data->estPose.y;
				wp_data_ring_head++;
				if( wp_data_ring_head >= PLOT_DATA_MAX )
					wp_data_ring_head = 0;
				if( wp_data_ring_cnt < PLOT_DATA_MAX )
					wp_data_ring_cnt++;
				else 
					wp_data_ring_cnt = PLOT_DATA_MAX;
			}
			flag_vehicleDirection_old = data->flag_slip;
		}
	}
	
}
*/
void GraphDrawerWP::writeSaveFile( void )
{
	create_wp.saveWPFile( );
}
