/*
 * Date : 2018.07.20
 * Update : 2019.08.22
 * Author : T.Hasegawa
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
#include <ssmtype/spur-odometry.h>
#include "detectObstacle.hpp"
#include "obp.hpp"
#include "urg.hpp"
#include "config.hpp"

// **************************************
// ******* 障害物検知の基本クラス ********
// *************************************
void detectObstacle_Base::setParameter( const config_property cnf )
{
	conf = cnf;
	
	for( int i = 0 ; i < NUM_AREA_TYPE ; i++ ){
		dec[ i ].p1.x = conf.dec_area[ i ].p1.x;
		dec[ i ].p1.y = conf.dec_area[ i ].p1.y;
		dec[ i ].p2.x = conf.dec_area[ i ].p2.x;
		dec[ i ].p2.y = conf.dec_area[ i ].p2.y;
		dec[ i ].p3.x = conf.dec_area[ i ].p3.x;
		dec[ i ].p3.y = conf.dec_area[ i ].p3.y;
		dec[ i ].p4.x = conf.dec_area[ i ].p4.x;
		dec[ i ].p4.y = conf.dec_area[ i ].p4.y;
		stop[ i ].p1.x = conf.stop_area[ i ].p1.x;
		stop[ i ].p1.y = conf.stop_area[ i ].p1.y;
		stop[ i ].p2.x = conf.stop_area[ i ].p2.x;
		stop[ i ].p2.y = conf.stop_area[ i ].p2.y;
		stop[ i ].p3.x = conf.stop_area[ i ].p3.x;
		stop[ i ].p3.y = conf.stop_area[ i ].p3.y;
		stop[ i ].p4.x = conf.stop_area[ i ].p4.x;
		stop[ i ].p4.y = conf.stop_area[ i ].p4.y;
		avoid[ i ].p1.x = conf.avoid_area[ i ].p1.x;
		avoid[ i ].p1.y = conf.avoid_area[ i ].p1.y;
		avoid[ i ].p2.x = conf.avoid_area[ i ].p2.x;
		avoid[ i ].p2.y = conf.avoid_area[ i ].p2.y;
		avoid[ i ].p3.x = conf.avoid_area[ i ].p3.x;
		avoid[ i ].p3.y = conf.avoid_area[ i ].p3.y;
		avoid[ i ].p4.x = conf.avoid_area[ i ].p4.x;
		avoid[ i ].p4.y = conf.avoid_area[ i ].p4.y;
	}
	printAreaData( );	// 確認用
	
	obp.status = TRAVELING;	// スタート時は走行モード
	setAreaType( 0 );
}
void detectObstacle_Base::printObstData( void )
{
	printf( "status=%d, x=%f[m], y=%f[m], th=%f[rad]\n", obp.status, obp.pos[ 0 ], obp.pos[ 1 ], obp.pos[ 2 ] );
}
void detectObstacle_Base::printAreaData( void )
{		
	for( int i = 0 ; i < NUM_AREA_TYPE ; i++ ){
		printf( "<TYPE #%d>\n", i );		
		printf( "%8.3f # P1 X [m] FOR DECELERATION AREA\n", dec[ i ].p1.x );
		printf( "%8.3f #    Y [m] \n", dec[ i ].p1.y );
		printf( "%8.3f # P2 X [m] FOR DECELERATION AREA\n", dec[ i ].p2.x );
		printf( "%8.3f #    Y [m] \n", dec[ i ].p2.y );
		printf( "%8.3f # P3 X [m] FOR DECELERATION AREA\n", dec[ i ].p3.x );
		printf( "%8.3f #    Y [m] \n", dec[ i ].p3.y );
		printf( "%8.3f # P4 X [m] FOR DECELERATION AREA\n", dec[ i ].p4.x );
		printf( "%8.3f #    Y [m] \n\n", dec[ i ].p4.y );
		printf( "%8.3f # P1 X [m] FOR EMERGENCY STOP AREA\n", stop[ i ].p1.x );
		printf( "%8.3f #    Y [m] \n", stop[ i ].p1.y );
		printf( "%8.3f # P2 X [m] FOR EMERGENCY STOP AREA\n", stop[ i ].p2.x );
		printf( "%8.3f #    Y [m] \n", stop[ i ].p2.y );
		printf( "%8.3f # P3 X [m] FOR EMERGENCY STOP AREA\n", stop[ i ].p3.x );
		printf( "%8.3f #    Y [m] \n", stop[ i ].p3.y );
		printf( "%8.3f # P4 X [m] FOR EMERGENCY STOP AREA\n", stop[ i ].p4.x );
		printf( "%8.3f #    Y [m] \n\n", stop[ i ].p4.y );
	}
}
// *******************************************
// ******* 2D平面での障害物検知クラス ********
// *******************************************
void detectObstacle_2D::chkObstacle( urg_fs *urg )
{
	bool stop_flag = false;
	bool dec_flag = false;
	unsigned int stop_counter = 0, dec_counter = 0;
	double dec_dist_min = 500.0, stop_dist_min = 500.0;
	double obp_x_stop = 0, obp_y_stop = 0, obp_x_dec = 0, obp_y_dec = 0;
	double distance = 0;
		
	for( int j = 0 ; j < urg->size ; j+=skip_point ){
		double x = urg->length[ j ] * cos( urg->angle[ j ] );
		double y = urg->length[ j ] * sin( urg->angle[ j ] );
		distance = sqrt( x*x + y*y );

		// 衝突回避のためのポイントデータの取得
		if( obp.status == STOP ){	// 回避中の判定
			if( ( avoid[ area_type ].p1.x <= x ) && ( x <= avoid[ area_type ].p3.x ) ){
				if( ( avoid[ area_type ].p1.y <= y ) && ( y <= avoid[ area_type ].p2.y ) ){
					stop_counter++;
					if( stop_counter >= pointNThre ){
						stop_flag = true;
					}
					if( stop_dist_min > distance ){
						stop_dist_min = distance;
						obp_x_stop = x;
						obp_y_stop = y;
					}
				}
			} else if( dec[ area_type ].p1.x <= x && x <= dec[ area_type ].p3.x ){
				if( dec[ area_type ].p1.y <= y && y <= dec[ area_type ].p2.y ){
					dec_counter++;
					if( dec_counter >= pointNThre ){
						dec_flag = true;
					}
					if( dec_dist_min > distance ){
						dec_dist_min = distance;
						obp_x_dec = x;
						obp_y_dec = y;
					}
				}
			}
		} else { // obp.status != STOP
			if( ( stop[ area_type ].p1.x <= x ) && ( x <= stop[ area_type ].p3.x ) ){	// 停止が最優先
				if( ( stop[ area_type ].p1.y <= y ) && ( y <= stop[ area_type ].p2.y ) ){
					stop_counter++;
					if( stop_counter >= pointNThre ){
						stop_flag = true;
					}
					if( stop_dist_min > distance ){
						stop_dist_min = distance;
						obp_x_stop = x;
						obp_y_stop = y;
					}
				}
			} else if( dec[ area_type ].p1.x <= x && x <= dec[ area_type ].p3.x ){		// 減速
				if( dec[ area_type ].p1.y <= y && y <= dec[ area_type ].p2.y ){
					dec_counter++;
					if( dec_counter >= pointNThre ){
						dec_flag = true;
					}
					if( dec_dist_min > distance ){
						dec_dist_min = distance;
						obp_x_dec = x;
						obp_y_dec = y;
					}
				}
			}
		}
	}

	if( stop_flag == true ){
		obp.status = STOP;
		obp.pos[ 0 ] = obp_x_stop;
		obp.pos[ 1 ] = obp_y_stop;
		obp.pos[ 2 ] = atan2( obp_x_stop, obp_y_stop );
	} else if( dec_flag == true ){
		obp.status = DECELERATION;
		obp.pos[ 0 ] = obp_x_dec;
		obp.pos[ 1 ] = obp_y_dec;
		obp.pos[ 2 ] = atan2( obp_x_dec, obp_y_dec );
	} else {
		obp.status = TRAVELING;
		obp.pos[ 0 ] = 0;
		obp.pos[ 1 ] = 0;
		obp.pos[ 2 ] = 0;
	}
}
// ***************************************************
// ******* 2D PCDを3D変換後の障害物検知クラス ********
// ***************************************************
void detectObstacle_3D::setParameter( const config_property cnf )
{
	detectObstacle_Base::setParameter( cnf );

	height = ( double )conf.urg3d.offset[ _Z ] / 1000.0;
	theta = DEG2RAD( conf.urg3d.rot[ _PITCH ] );
	setSkipPoint( 1 );
	setPointNThre( 5 );
}
	
typedef struct {
	unsigned int size;
	double x[ 300 ], y[ 300 ], z[ 300 ];
	double average[ 3 ];
} ClusterType;
static double length_between_points = 0.1;	// ポイント間距離
static double diff_tx = 0.3;	// 地面ラインからの障害物までの距離(x値)の閾値
static double diff_tz = 0.05;	// 地面ラインからの障害物までの距離(z値)の閾値
void detectObstacle_3D::chkObstacle( urg_fs *urg )
{
	bool flag_detectObp = false;
	bool flag_startChecking_obstacle = false;
	ClusterType cluster[ 30 ];
	unsigned int cluster_cnt = 0, pcnt = 0;
	
	for( int i = 1 ; i < urg->size ; i += skip_point ){
		// 座標変換
		double xx = urg->length[ i ] * cos( urg->angle[ i ] );
		double yy = urg->length[ i ] * sin( urg->angle[ i ] );
		double tx = xx * cos( theta );
		double ty = yy;
		double tz = -1.0 * xx * sin( theta ) + height;

		if( ( stop[ area_type ].p1.x <= tx ) && 	// 停止が最優先
			( stop[ area_type ].p1.y <= ty ) && ( ty <= stop[ area_type ].p2.y ) ){
		
			// ポイント間距離を余弦定理で算出
			double dL = sqrt( urg->length[ i-1 ]*urg->length[ i-1 ] + urg->length[ i ]*urg->length[ i ] 
								- 2.0 * urg->length[ i-1 ]*urg->length[ i ] * cos( fabs( urg->angle[ i ] - urg->angle[ i-1 ] ) ) );
//			printf("%d %f\n",i, dL );
			
			if( dL <= length_between_points ){	// あるオブジェクトのクラスタリング開始
				if( !flag_startChecking_obstacle )
					flag_startChecking_obstacle = true;

				cluster[ cluster_cnt ].x[ pcnt ] = tx;
				cluster[ cluster_cnt ].y[ pcnt ] = ty;
				cluster[ cluster_cnt ].z[ pcnt ] = tz;
				pcnt++;
				if( pcnt >= 300 ) pcnt = 299;

			} else {	// あるオブジェクトのクラスタリング終了
				flag_startChecking_obstacle = false;
				cluster[ cluster_cnt ].size = pcnt + 1;
				double sum[ 3 ] = { 0 };
				for( int t = 0 ; t < cluster[ cluster_cnt ].size ; t++ ){
					sum[ 0 ] += cluster[ cluster_cnt ].x[ t ];
					sum[ 1 ] += cluster[ cluster_cnt ].y[ t ];
					sum[ 2 ] += cluster[ cluster_cnt ].z[ t ];
				}
				for( int t = 0 ; t < 3 ; t++ ){
					cluster[ cluster_cnt ].average[ t ] = sum[ t ] / ( double )cluster[ cluster_cnt ].size;
				}
				pcnt = 0;
				cluster_cnt++;
				if( cluster_cnt >= 30 ) cluster_cnt = 29;
			}
			
//			printf("%d %f %f %f %f\n",i, tz_sum, tz_average, tz, dL );
		}
	}
	// あるオブジェクトのクラスタリングが終了せずループを抜けた場合の処理
	if( flag_startChecking_obstacle ){
		flag_startChecking_obstacle = false;
		cluster[ cluster_cnt ].size = pcnt + 1;
		double sum[ 3 ] = { 0 };
		for( int t = 0 ; t < cluster[ cluster_cnt ].size ; t++ ){
			sum[ 0 ] += cluster[ cluster_cnt ].x[ t ];
			sum[ 1 ] += cluster[ cluster_cnt ].y[ t ];
			sum[ 2 ] += cluster[ cluster_cnt ].z[ t ];
		}
		for( int t = 0 ; t < 3 ; t++ ){
			cluster[ cluster_cnt ].average[ t ] = sum[ t ] / ( double )cluster[ cluster_cnt ].size;
		}
		pcnt = 0;
		cluster_cnt++;
		if( cluster_cnt >= 30 ) cluster_cnt = 30;
	}
	
//	printf( "cluster_cnt=%d\n",cluster_cnt);
	double max_dist_x = -500, min_dist_x = 500;
	int max_id, min_id;
	for( int i = 0 ; i < cluster_cnt ; i++ ){
		if( cluster[ i ].size < pointNThre ) continue;
		double dist = sqrt( cluster[ i ].average[ 0 ]*cluster[ i ].average[ 0 ] + cluster[ i ].average[ 1 ]*cluster[ i ].average[ 1 ] + cluster[ i ].average[ 2 ]*cluster[ i ].average[ 2 ] );
//		printf("[%d] cnt=%d, x=%f, y=%f, z=%f, d=%f\n", i, cluster[ i ].size, cluster[ i ].average[ 0 ], cluster[ i ].average[ 1 ], cluster[ i ].average[ 2 ], dist );
		if( min_dist_x > cluster[ i ].average[ 0 ] ){	// おそらく、最短距離の障害物のx値
			min_dist_x = cluster[ i ].average[ 0 ];
			min_id = i;
		}
		if( max_dist_x > cluster[ i ].average[ 0 ] ){	// おそらく、地面までのxの値
			max_dist_x = cluster[ i ].average[ 0 ];
			max_id = i;
		}
	}
//	printf("id max=%d, min=%d\n", max_id, min_id );
	if( ( cluster[ max_id ].average[ 0 ] - cluster[ min_id ].average[ 0 ] ) > diff_tx ){
		if( ( cluster[ min_id ].average[ 2 ] - cluster[ max_id ].average[ 2 ] ) > diff_tz ){
			flag_detectObp = true;
		}
	}

	if( flag_detectObp == true ){
		obp.status = STOP;
		obp.pos[ 0 ] = cluster[ min_id ].average[ 0 ];
		obp.pos[ 1 ] = cluster[ min_id ].average[ 1 ];
		obp.pos[ 2 ] = cluster[ min_id ].average[ 2 ];
	} else {
		obp.status = TRAVELING;
		obp.pos[ 0 ] = 0;
		obp.pos[ 1 ] = 0;
		obp.pos[ 2 ] = 0;
	}

}
