/*
 * Manage Way Points
 * 
 * Date   : 2021.03.27
 * Author : T.Hasegawa
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>

#include <iostream>
#include <vector>
#include <limits>
#include <fstream>

#include <ssm.hpp>
#include "framework.hpp"
#include "wp-mgr.hpp"
#include "config.hpp"
#include "utility.hpp"

#ifdef Use_Follow_Circle
static void calcCenterOfCircle( wp_gl *w, int cnt );
#else
#endif
// *************************************************************************************
// ***************** 基本クラス。WPファイルから読み込んで順次出力 ************************
// *************************************************************************************
void WpMgr_Base::initilize( const config_property c, unsigned int id, ROBOT_STATUS status )
{
//	robot_status = status;
	robot_status = NAVI;
	flag_update_wp = true;	// WPを更新
	conf = c;
	
	char filename[ STRLEN ];
	sprintf( filename, "%s", conf.wp_info.filename );
	printf( "\nWP FILE = %s\n", filename );		// 確認用

	FILE *fp_wp = isValidFile( filename, "r", "dat" );
	wp_num = atoi( getWord( fp_wp ) );
	if( ( wp = new wp_gl[ wp_num ] ) == NULL ){
		std::cerr << "Cannot allocate WP variables." << std::endl;
		exit( EXIT_FAILURE );
	}

//	printf( "wp_num=%d\n", wp_num );
	for( int i = 0; i < wp_num ; i++ ){
//		printf( "%d,", i );
		wp[ i ].id = atoi( getWord( fp_wp ) );
		wp[ i ].x = atof( getWord( fp_wp ) );
		wp[ i ].y = atof( getWord( fp_wp ) );
		double v = atof( getWord( fp_wp ) );
		if( v > conf.navi.fvel ) v = conf.navi.fvel;
		else if( v  < conf.navi.bvel ) v = conf.navi.bvel;
		if( i == 0 ){
			wp[ i ].v = 0;
		} else {
			wp[ i ].v = v;
		}
		wp[ i ].flag_cut = atoi( getWord( fp_wp ) );
		wp[ i ].area_type = atoi( getWord( fp_wp ) );
		wp[ i ].gain_id = atoi( getWord( fp_wp ) );
#ifdef Use_Follow_Circle
		wp[ i ].circle[ _R ] = atof( getWord( fp_wp ) );
//		printf( "[%d] R=%f\n", i, wp[ i ].circle[ _R ] );
#endif
	}
	for( int i = 0; i < wp_num ; i++ ){
		if( i == 0 ){
			wp[ i ].theta = atan2( wp[ i+1 ].y - wp[ i ].y, wp[ i+1 ].x - wp[ i ].x );
		} else {
			wp[ i ].theta = atan2( wp[ i ].y - wp[ i-1 ].y, wp[ i ].x - wp[ i-1 ].x );
		}
	}
#ifdef Use_Follow_Circle
		calcCenterOfCircle( wp, wp_num );
		//for( int i = 0; i < wp_num ; i++ ){
			//printf( "[%d]x=%f, y=%f\n", i, wp[ i ].circle[ _X ], wp[ i ].circle[ _Y ] );
////			printf( "%d %f %f\n", i, wp[ i ].circle[ _X ], wp[ i ].circle[ _Y ] );
			//printf( "[%d] R=%f\n", i, wp[ i ].circle[ _R ] );
		//}
#else
#endif
	fclose( fp_wp );
	
	offset_gl[ _X ] = 0.0;
	offset_gl[ _Y ] = 0.0;
			
	printAllWP( );		// 確認用
	setStartWPid( id );
}
#ifdef Use_Follow_Circle
static void calcCenterOfCircle( wp_gl *w, int cnt )
{
	double p1[ 2 ], p2[ 2 ], p3[ 2 ];
	for( int i = 0; i < cnt ; i++ ){
		if( w[ i ].circle[ _R ] == 0 ) continue;	// 円弧でなければスキップ
		p1[ _X ] = w[ i ].x;	// WP(i)を登録
		p1[ _Y ] = w[ i ].y;
		p2[ _X ] = w[ i+1 ].x;	// WP(i+1)を登録
		p2[ _Y ] = w[ i+1 ].y;
//		printf("x1=%f,y1=%f,,x2=%f,y2=%f\n",p1[0],p1[1],p2[0],p2[1]);
		if( w[ i+1 ].circle[ _R ] == 0 ){	// 1つ先のWPが円弧でない場合
			if( w[ i-1 ].circle[ _R ] == 0 ){
				fprintf( stderr, "Error. Must check WP[%d]. in calcCenterOfCircle\n", i );
				exit( EXIT_FAILURE );
			} else {
				p1[ _X ] = w[ i-1 ].x;	// WP(i-1)を登録
				p1[ _Y ] = w[ i-1 ].y;
				p2[ _X ] = w[ i ].x;	// WP(i)を登録
				p2[ _Y ] = w[ i ].y;
			}
		}
		// 中点Pを算出
		p3[ _X ] = ( p1[ _X ] + p2[ _X ] ) / 2.0;
		p3[ _Y ] = ( p1[ _Y ] + p2[ _Y ] ) / 2.0;
		// 各辺の長さ
		double dist_a = sqrt( ( p3[ _X ] - p2[ _X ] ) * ( p3[ _X ] - p2[ _X ] ) + ( p3[ _Y ] - p2[ _Y ] ) * ( p3[ _Y ] - p2[ _Y ] ) );
		double dist_b = sqrt( w[ i ].circle[ _R ] * w[ i ].circle[ _R ] - ( dist_a * dist_a ) );
//		printf("a=%f, b=%f\n", dist_a, dist_b );
		// 角度
//		double fai = atan2( ( p3[ _Y ] - p1[ _Y ] ), ( p3[ _X ] - p1[ _X ] ) );
		double fai = atan2( ( p3[ _X ] - p1[ _X ] ), ( p3[ _Y ] - p1[ _Y ] ) );
		// 円の中心位置を算出
		if( w[ i ].circle[ _R ] < 0 ){
			w[ i ].circle[ _X ] = p3[ _X ] + dist_b * cos( fai );
			w[ i ].circle[ _Y ] = p3[ _Y ] - dist_b * sin( fai );
		} else {
			w[ i ].circle[ _X ] = p3[ _X ] - dist_b * cos( fai );
			w[ i ].circle[ _Y ] = p3[ _Y ] + dist_b * sin( fai );
		}
//		printf( "[%d]x=%f, y=%f\n", i, w[ i ].circle[ _X ], w[ i ].circle[ _Y ] );
//		printf( "%d %f %f\n", i, w[ i ].circle[ _X ], w[ i ].circle[ _Y ] );
//		printf( "[%d] R=%f\n", i, w[ i ].circle[ _R ] );
	}
}
#else
#endif
// ********************* getWPの関連 *************************
wp_gl WpMgr_Base::getWP( localizer *odm )
{
	calcWPOffset( );
	return getWPcurrent( );
}
wp_gl WpMgr_Base::getWPcurrent( void )	// 通常のWP
{
	flag_update_wp = false;	// WPを更新しない
	return wp[ wp_cnt ];
}

// ********************* 開始WPの設定 *************************
void WpMgr_Base::setStartWPid( unsigned int i )
{
	wp_cnt = i - 1;
	if( wp_cnt >= wp_num ){
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: WP ID for start is wrong\n" );
	}
}
// ********************* ゴール判定 *************************
bool WpMgr_Base::chkGoal( void )
{
	if( wp_cnt >= wp_num ){
		Gprint( "\nGoal announced from WpMgr \n\n" );
		wp_cnt = wp_num - 1;
		return true;
	} else
		return false;
}
// ********************* 表示関係 *************************
void WpMgr_Base::printAllWP( void )
{
	printf( "All WP\n" );
	for( int i = 0; i < wp_num ; i++ ){
		printf( "[%2d] (%8.4f[m], %8.4f[m], %8.4f[rad]), v=%3.1f, c=%d, t=%d\n", wp[ i ].id, wp[ i ].x, wp[ i ].y, wp[ i ].theta, wp[ i ].v, wp[ i ].flag_cut, wp[ i ].area_type );
	}
	printf( "\n" );
}
void WpMgr_Base::printWPcurrent( void )
{
	printf( "[%2d] (%8.4f[m], %8.4f[m], %8.4f[rad]) v=%3.1f[m/s], c=%d, t=%d\n", wp[ wp_cnt ].id, wp[ wp_cnt ].x, wp[ wp_cnt ].y, wp[ wp_cnt ].theta, wp[ wp_cnt ].v, wp[ wp_cnt ].flag_cut, wp[ wp_cnt ].area_type );
}
// 今回は指定したWPを表示。（WpMgr_Proactiveクラスで、先読みWPと車両代表点におけるWPの両方を表示）
void WpMgr_Base::printWP( wp_gl *w )
{
	printf( "[%2d] (%8.4f[m], %8.4f[m], %8.4f[rad]) v=%3.1f[m/s], c=%d, t=%d\n", w->id, w->x, w->y, w->theta, w->v, w->flag_cut, w->area_type );
}
// ********************* OverLine関連 *************************
bool WpMgr_Base::chkOverLine( localizer *odm )
{
	double over_line_x = wp[ wp_cnt ].x - offset_gl[ _X ];
	double over_line_y = wp[ wp_cnt ].y - offset_gl[ _Y ];
	double over_line_th = wp[ wp_cnt ].theta + M_PI/2.0;

	double distance = ( over_line_x - odm->estPose.x ) * sin( over_line_th ) - ( over_line_y - odm->estPose.y ) * cos( over_line_th );	// MUST check

	if( distance < 0 ) return true;
	
	return false;
}
// ********************* WP用offsetの計算とセット ***********************
void WpMgr_Base::calcWPOffset( void )
{
	if( wp_cnt == wp_num - 1 ){	// Goal時
		double offset_lc = 0.15;
		offset_gl[ _X ] = offset_lc * cos( wp[ wp_cnt ].theta );
		offset_gl[ _Y ] = offset_lc * sin( wp[ wp_cnt ].theta );
		fprintf( stderr, "[WP%2d] offset_lc=%8.4f[m]\n", wp[ wp_cnt ].id, offset_gl[ _X ] );	// 確認用
		
	} else {

		double vel_tmp = wp[ wp_cnt ].v;
		if( vel_tmp == 0 ) vel_tmp = SIGN( wp[ wp_cnt-1 ].v ) * 0.01;	// ゼロでの割り算を回避
		if( ( wp[ wp_cnt+1 ].v / vel_tmp ) >= 0 ){
			// 旋回半径からoffsetを計算
			double dth = trans_q( wp[ wp_cnt+1 ].theta - wp[ wp_cnt ].theta );
			double offset_lc = conf.wp_info.TR * tan( dth / 2 );
			offset_lc = fabs( offset_lc );
		
//			offset_lc += fabs( wp[ wp_cnt ].v ) * 1;	// 要チェック。1秒分遅れると予想
			double dx = wp[ wp_cnt+1 ].x - wp[ wp_cnt ].x;
			double dy = wp[ wp_cnt+1 ].y - wp[ wp_cnt ].y;
			double dist = sqrt( dx*dx + dy*dy ); // 次のWPまでの距離
			double offset_lc_tmp = fabs( wp[ wp_cnt ].v ) * 1;	// 要チェック。1秒分遅れると予想
			if( ( offset_lc + offset_lc_tmp ) < ( dist - 0.5 ) ){
				offset_lc += offset_lc_tmp;
			} else if( offset_lc > ( dist - 0.5 ) ){
				offset_lc = dist * 0.4; // distの４割
			}

			offset_gl[ _X ] = offset_lc * cos( wp[ wp_cnt ].theta );
			offset_gl[ _Y ] = offset_lc * sin( wp[ wp_cnt ].theta );
			fprintf( stderr, "[WP%2d] offset_lc=%8.4f[m], dth=%8.4f[rad], TR=%8.4f[m]\n", wp[ wp_cnt ].id, offset_lc, dth, conf.wp_info.TR );	// 確認用

		} else {	// 前後退の切替時
			double offset_lc = 0.15;
			offset_gl[ _X ] = offset_lc * cos( wp[ wp_cnt ].theta );
			offset_gl[ _Y ] = offset_lc * sin( wp[ wp_cnt ].theta );
			if( wp[ wp_cnt+1 ].v < 0 ){
				fprintf( stderr, "[WP%2d] offset_lc=%8.4f[m], Backward\n", wp[ wp_cnt ].id, offset_lc );	// 確認用
			} else {
				fprintf( stderr, "[WP%2d] offset_lc=%8.4f[m], Forward\n", wp[ wp_cnt ].id, offset_lc );	// 確認用
			}
		}
	}
}
// ******************************************************************
// **************** ポジショニング用WPクラス *************************
// ******************************************************************
void WpMgr_Positioning::initilize( const config_property c, unsigned int id, ROBOT_STATUS status )
{
	WpMgr_Base::initilize( c, id, status );

	robot_status = status;
	
	if( robot_status == POSITIONING ){
		flag_positioning_forward = true;	// ture:前進, false:後退
		length4positioning = 5.0;		// 調整距離[m]を設定
		velocity4positioning = 0.6;		// ポジショニング中の速度設定[m/s]
		positioning_loop_num = 4;

		calcWP4Positioning( );
	}
}
// ********************* 表示関係 *************************
// 今回は指定したWPを表示。（WpMgr_Proactiveクラスで、先読みWPと車両代表点におけるWPの両方を表示）
void WpMgr_Positioning::printWP( wp_gl *w )
{
	WpMgr_Base::printWP( w );
}
// ********************* getWPの関連 *************************
wp_gl WpMgr_Positioning::getWP( localizer *odm )
{
	flag_update_wp = false;	// WPを更新しない
	if( robot_status == POSITIONING ){
		return getWP4Positioning( odm );
	} else {
		calcWPOffset( );
		return getWPcurrent( );
	}
}
// ********************* OverLine関連 *************************
bool WpMgr_Positioning::chkOverLine( localizer *odm )
{
	double over_line_x;
	double over_line_y;
	double over_line_th;

	if( robot_status == NAVI ){
		over_line_x = wp[ wp_cnt ].x - offset_gl[ _X ];
		over_line_y = wp[ wp_cnt ].y - offset_gl[ _Y ];
		over_line_th = wp[ wp_cnt ].theta + M_PI/2.0;
	
	} else if( robot_status == POSITIONING ){
		if( !flag_positioning_forward ){	// getWPで命令後にflagが反転するので、!をつける
			// 前進走行の場合
//			printf( "forward_overline\n" );
			over_line_x = wp_positioning[ 1 ].x - offset_gl[ _X ];
			over_line_y = wp_positioning[ 1 ].y - offset_gl[ _Y ];
			over_line_th = wp_positioning[ 1 ].theta + M_PI/2.0;

		} else {	// バック走行の場合
//			printf( "backward_overline\n" );
			over_line_x = wp_positioning[ 0 ].x - offset_gl[ _X ];
			over_line_y = wp_positioning[ 0 ].y - offset_gl[ _Y ];
			over_line_th = wp_positioning[ 0 ].theta + M_PI/2.0;
		}
		
	}
	
	double distance = ( over_line_x - odm->estPose.x ) * sin( over_line_th ) - ( over_line_y - odm->estPose.y ) * cos( over_line_th );	// MUST check
	if( distance < 0 ) return true;
	
	return false;
}
// **************** ポジショニング用のgetWP **************************
void WpMgr_Positioning::calcWP4Positioning( void )
{
	// 後退走行用WP
	wp_positioning[ 0 ].id = 1000 + wp[ wp_cnt-1 ].id;
	wp_positioning[ 0 ].x = wp[ wp_cnt-1 ].x;
	wp_positioning[ 0 ].y = wp[ wp_cnt-1 ].y;
	wp_positioning[ 0 ].v = -1.0 * velocity4positioning;
	wp_positioning[ 0 ].flag_cut = 0;
	wp_positioning[ 0 ].area_type = wp[ wp_cnt-1 ].area_type;
	// 前進走行用WP
	wp_positioning[ 1 ].id = 1000 + wp[ wp_cnt ].id;
	wp_positioning[ 1 ].x = length4positioning * cos( wp[ wp_cnt-1 ].theta ) + wp[ wp_cnt-1 ].x;
	wp_positioning[ 1 ].y = length4positioning * sin( wp[ wp_cnt-1 ].theta ) + wp[ wp_cnt-1 ].y;
	wp_positioning[ 1 ].v = velocity4positioning;
	wp_positioning[ 1 ].flag_cut = 0;
	wp_positioning[ 1 ].area_type = wp[ wp_cnt ].area_type;
	
	wp_positioning[ 0 ].theta = atan2( wp_positioning[ 0 ].y - wp_positioning[ 1 ].y, wp_positioning[ 0 ].x - wp_positioning[ 1 ].x );
	wp_positioning[ 1 ].theta = atan2( wp_positioning[ 1 ].y - wp_positioning[ 0 ].y, wp_positioning[ 1 ].x - wp_positioning[ 0 ].x );

	Gprint( "\nWP for Positioning " );
	printf( ": %d\n", robot_status );
	for( int i = 0 ; i < 2 ; i++ ){
		printf( "[%2d] (%8.4f[m], %8.4f[m], %8.4f[rad]) v=%3.1f[m/s], c=%d, t=%d\n", 
				wp_positioning[ i ].id, wp_positioning[ i ].x, wp_positioning[ i ].y, wp_positioning[ i ].theta, wp_positioning[ i ].v, wp_positioning[ i ].flag_cut, wp_positioning[ i ].area_type );
	}
	printf( "\n" );
	
}
static int positioning_loop_cnt = 1;
wp_gl WpMgr_Positioning::getWP4Positioning( localizer *odm )
{
	flag_update_wp = false;	// WPを更新しない
	
	// とりあえず、ここでオフセットを設定しておく
//	offset_gl[ _X ] = 0.3;
//	offset_gl[ _Y ] = 0.3;
	double offset_lc = 0.3;
	offset_gl[ _X ] = offset_lc * cos( wp[ wp_cnt ].theta );
	offset_gl[ _Y ] = offset_lc * sin( wp[ wp_cnt ].theta );
		
	if( flag_positioning_forward ){
		flag_positioning_forward = false;
		// 位置・姿勢の範囲を確認
		double len, ang;
		double tx = wp_positioning[ 0 ].x - odm->estPose.x;
		double ty = wp_positioning[ 0 ].y - odm->estPose.y;
		len = sqrt( tx*tx + ty*ty );
		ang = fabs( wp_positioning[ 1 ].theta - odm->estPose.theta );
		if( positioning_loop_cnt >= 2 ){
			if( ( ( len <= 0.1 ) && ( ang <= DEG2RAD( 3.0 ) ) ) || ( positioning_loop_cnt >= positioning_loop_num ) ){
				robot_status = NAVI;
				positioning_loop_cnt = 1;
				flag_positioning_forward = true;
				return wp[ wp_cnt ];
			}
		}
		// とりあえず、ここでオフセットを設定しておく
		double offset_lc = 0.3;
		offset_gl[ _X ] = offset_lc * cos( wp_positioning[ 1 ].theta );
		offset_gl[ _Y ] = offset_lc * sin( wp_positioning[ 1 ].theta );
		
		return wp_positioning[ 1 ];
		
	} else {
		flag_positioning_forward = true;
		positioning_loop_cnt++;
		
		// とりあえず、ここでオフセットを設定しておく
		double offset_lc = 0.3;
		offset_gl[ _X ] = offset_lc * cos( wp_positioning[ 0 ].theta );
		offset_gl[ _Y ] = offset_lc * sin( wp_positioning[ 0 ].theta );
		
		return wp_positioning[ 0 ];
		
	}
}
// *********************************************************
// **************** 先読みWPを追加 *************************
// *********************************************************
void WpMgr_Proactive::initilize( const config_property c, unsigned int id, ROBOT_STATUS status )
{
	WpMgr_Positioning::initilize( c, id, status );
	proactive_wp_cnt = wp_cnt;

}
// 先読みWPのgetWP
wp_gl WpMgr_Proactive::getWP4Proactive( localizer *odm )
{
	flag_update_wp = false;	// WPを更新しない
	
	// このオフセットは基本的に使用しない
	offset_gl[ _X ] = 0;
	offset_gl[ _Y ] = 0;
		
	unsigned int wp_skip_cnt = 0;
	double estDist = odm->estPose.v * proactive_time;//2.0;	// 2.0秒先の距離を予測
	double tx = wp[ wp_cnt ].x - odm->estPose.x;
	double ty = wp[ wp_cnt ].y - odm->estPose.y;
	double dist = sqrt( tx*tx + ty*ty );
	if( dist > estDist ){
		wp_skip_cnt = 0;
	} else {
		for( wp_skip_cnt = 1 ; ; wp_skip_cnt++ ){
			tx =  wp[ wp_cnt + wp_skip_cnt ].x - wp[ wp_cnt + wp_skip_cnt - 1 ].x;
			ty =  wp[ wp_cnt + wp_skip_cnt ].y - wp[ wp_cnt + wp_skip_cnt - 1 ].y;
			double tmp = sqrt( tx*tx + ty*ty );
			dist += tmp;
			if( dist > estDist ) break;
		}
	}
	// 前後退の切替時
	if( SIGN( wp[ wp_cnt + wp_skip_cnt ].v ) != SIGN( wp[ wp_cnt ].v ) ){
		for( int i = 0 ; i < wp_skip_cnt ; i++ ){
			if( SIGN( wp[ wp_cnt + i ].v ) == SIGN( wp[ wp_cnt ].v ) ){
				wp_skip_cnt = i;
//				offset_gl[ _X ] = 0.3;	// 要検討
//				offset_gl[ _Y ] = 0.3;
				if( ( ( wp_cnt + wp_skip_cnt ) == ( wp_cnt ) ) && ( SIGN( wp[ wp_cnt + 1 ].v ) != SIGN( wp[ wp_cnt ].v ) ) ){
					double offset_lc = 0.3;
					offset_gl[ _X ] = offset_lc * cos( wp[ wp_cnt ].theta );
					offset_gl[ _Y ] = offset_lc * sin( wp[ wp_cnt ].theta );
				}
			}
		}
	}
//	printf("      [%d]offset_gl[ _X ]=%f, offset_gl[ _Y ]=%f\n", wp[ wp_cnt + wp_skip_cnt ].id, offset_gl[ _X ], offset_gl[ _Y ] );
//	printf("      v=%f, dir=%d\n", odm->estPose.v, odm->dir );
#ifdef Update_Proactive_WP_AnyTime
	proactive_wp_cnt = wp_cnt + wp_skip_cnt;
#else
	if( proactive_wp_cnt < ( wp_cnt + wp_skip_cnt ) ){
		proactive_wp_cnt = wp_cnt + wp_skip_cnt;
	}
#endif
	return wp[ proactive_wp_cnt ];
}
// ********************* getWPの関連 *************************
wp_gl WpMgr_Proactive::getWP( localizer *odm )
{
	flag_update_wp = false;	// WPを更新しない
	if( robot_status == NAVI ){
		return getWP4Proactive( odm );
	} else if( robot_status == POSITIONING ){
		return getWP4Positioning( odm );
	} else {
		calcWPOffset( );
		return getWPcurrent( );
	}
}
// 先読みWPと車両代表点におけるWPの両方を表示
void WpMgr_Proactive::printWP( wp_gl *w )
{
	printf( "[%2d(%2d)] (%8.4f[m], %8.4f[m], %8.4f[rad]) v=%3.1f[m/s], c=%d, t=%d\n", w->id, wp[ wp_cnt ].id, w->x, w->y, w->theta, wp[ wp_cnt ].v, wp[ wp_cnt ].flag_cut, wp[ wp_cnt ].area_type );
}
// ********************* OverLine関連 *************************
bool WpMgr_Proactive::chkOverLine( localizer *odm )
{
	return WpMgr_Positioning::chkOverLine( odm );
}
