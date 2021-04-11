/*
 * Date : 2019.09.30
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "GraphDrawerWP.hpp"
#include "utility.hpp"
#include <math.h>
#include "wp.hpp"

using namespace std;

void GraphDrawerWP::setParameters( const config_property c, bool f, const char *file )
{
	conf = c;
	flag_wpfile_config = f;
	if( flag_wpfile_config ){
		wp_mgr.initilize( conf, 1, NAVI );
	} else {
		sprintf( conf.wp_info.filename, "%s", file );
		printf( "%s\n", conf.wp_info.filename );
		wp_mgr.initilize( conf, 1, NAVI );
	}
	wp_size = wp_mgr.getNumWP( );
	// バック走行があるかの確認
	for( int i = 0; i < wp_mgr.getNumWP( ) ; i++ ){
		if( wp_mgr.getWPvel( i ) < 0 ) flag_red4backward = true;
	}
	// WPの表示
	wp_mgr.printAllWP( );
	// WPが適正かどうかの確認
	checkWP( );
}
void GraphDrawerWP::checkWP( void )
{
	// 要チェック 
	conf.cntl.clip_tgtSteering = DEG2RAD( conf.cntl.clip_tgtSteering );
	conf.wp_info.TR = conf.robot_info.wheelbase / tan( conf.cntl.clip_tgtSteering );
	printf( "conf.wp_info.TR=%f\n", conf.wp_info.TR );

	for( int i = 1; i < wp_mgr.getNumWP( ) ; i++ ){
		double dx = wp_mgr.getWPx( i ) - wp_mgr.getWPx( i-1 );
		double dy = wp_mgr.getWPy( i ) - wp_mgr.getWPy( i-1 );
		double dist = sqrt( dx*dx + dy*dy );

		printf( "[WP%2d-WP%2d] dist=%8.4f[m], v=%8.4f[m]\n", wp_mgr.getWPid( i ), wp_mgr.getWPid( i-1 ), dist, wp_mgr.getWPvel( i ) );
	}

}

#define RANGE 20	// 描画範囲[m]
static bool flag_first_loop = true;
void GraphDrawerWP::updateWP( void )
{
	if( flag_first_loop ){
		wp_size = 1;
		flag_red4backward = false;
		flag_first_loop = false;
	} else {
		wp_size++;
		if( wp_size >= wp_mgr.getNumWP( ) ) wp_size = wp_mgr.getNumWP( );
	}
	// バック走行の確認（赤色で表示）
	if( wp_mgr.getWPvel( wp_size - 1 ) < 0 ) flag_red4backward = true;
	// 描画範囲
	setRange( wp_mgr.getWPx( wp_size-1 )-RANGE, wp_mgr.getWPx( wp_size-1 )+RANGE, wp_mgr.getWPy( wp_size-1 )-RANGE, wp_mgr.getWPy( wp_size-1 )+RANGE );

}

void GraphDrawerWP::drawGraph( void )
{
	// Draw ID for WP
	if( wp_size > 1 ){
		for( int i = 1 ; i < wp_size ; i++ ){
			fprintf( gp, "set label \"\%2d\" at %lf+0.1,%lf+0.1\n", wp_mgr.getWPid( i ), wp_mgr.getWPx( i ), wp_mgr.getWPy( i ) );
		}
	}
	
	// WPの描画設定
	if( wp_size > 1 ){
		fprintf( gp, "p " );
		fprintf( gp, " '-' pt 5 ps 0.8 lc rgb 'black' t 'WP(+)', " );
		if( flag_red4backward )
			fprintf( gp, " '-' pt 5 ps 0.8 lc rgb 'red' t 'WP(-)', " );
	}
	if( wp_size > 1 ){
		fprintf( gp, " '-' w l lc rgb 'green'" );
	}
	fprintf( gp, "\n" );
	
	// WPの描画
	if( wp_size > 1 ){
		for( int i = 0 ; i < wp_size ; i++ ){
			if( wp_mgr.getWPvel( i ) > 0 )
				fprintf( gp, "%lf %lf\n", wp_mgr.getWPx( i ), wp_mgr.getWPy( i ) );
		}
		fprintf( gp, "e\n" );
	}
	if( wp_size > 1 && flag_red4backward ){
		for( int i = 0 ; i < wp_size ; i++ ){
			if( wp_mgr.getWPvel( i ) < 0 )
				fprintf( gp, "%lf %lf\n", wp_mgr.getWPx( i ), wp_mgr.getWPy( i ) );
		}
		fprintf( gp, "e\n" );
	}
	if( wp_size > 1 ){
		for( int i = 1 ; i < wp_size ; i++ ){
			fprintf( gp, "%lf %lf\n%lf %lf\n\n", wp_mgr.getWPx( i-1 ), wp_mgr.getWPy( i-1 ), wp_mgr.getWPx( i ), wp_mgr.getWPy( i ) );
		}
		fprintf( gp, "e\n" );
	}
	fflush( gp );
}
