 /*
  * Date : 2018.07.09
  * Update : 2019.01.12
  * Update : 2019.02.01
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_HPP_
#define _GRAPH_DRAWER_HPP_

#include <stdio.h>
#include "config.hpp"

class GraphDrawer
{
protected:
	FILE *gp;               // gnuplotへのパイプ
	double xmin;            // 描画範囲[m]
	double xmax;
	double ymin;
	double ymax;
	double aspectR;         // xy比

public:
	GraphDrawer( void ) : gp( nullptr ), xmin( -10 ), xmax( 10 ), ymin( -10 ), ymax( 10 ), aspectR( -1.0 ) {  }
	~GraphDrawer( void )
	{
		finishGnuplot( );
	}

	void initGnuplot( void )
	{
		gp = popen( "gnuplot", "w" );		// パイプオープン.Linux
		setGrid( );
		setAspectRatio( -0.9 );		// x軸とy軸の比（負にすると中身が一定）
	}

	void finishGnuplot( void )
	{
		if( gp != nullptr )
			pclose( gp );
	}

	void setAspectRatio( double a )
	{
		aspectR = a;
		fprintf( gp, "set size ratio %lf\n", aspectR );
	}

	void setRange( double R )		// 描画範囲をR四方にする
	{
		xmin = ymin = -R;
		xmax = ymax = R;
		fprintf( gp, "set xrange [%lf:%lf]\n", xmin, xmax );
		fprintf( gp, "set yrange [%lf:%lf]\n", ymin, ymax );
	}

	void setRange( double xR, double yR )		// 描画範囲を±xR、±yRにする
	{
		xmin = -xR;
		xmax = xR;
		ymin = -yR; 
		ymax = yR;
		fprintf( gp, "set xrange [%lf:%lf]\n", xmin, xmax );
		fprintf( gp, "set yrange [%lf:%lf]\n", ymin, ymax );
	}

	void setRange( double xm, double xM, double ym, double yM )		// 描画範囲を全部指定
	{
		xmin = xm;
		xmax = xM;
		ymin = ym; 
		ymax = yM;
		fprintf( gp, "set xrange [%lf:%lf]\n", xmin, xmax );
		fprintf( gp, "set yrange [%lf:%lf]\n", ymin, ymax );
	}
	void setGrid( void )
	{
		fprintf( gp, "set grid\n" );
	}
	
	virtual void drawGraph( void ) = 0;
};

class GraphDrawer_RobotBase : public GraphDrawer
{
protected:
	double robot[ 5 ][ 2 ];
	void fprintf_robot( double *edges, int edge_num, double *pos )
	{
		for( int i = 0; i <= edge_num; i++ ){
			int ii = i % edge_num;

			double x = edges[ ii * 2 ] * cos( pos[ 2 ] ) - edges[ ii * 2 + 1 ] * sin( pos[ 2 ] ) + pos[ 0 ];
			double y = edges[ ii * 2 ] * sin( pos[ 2 ] ) + edges[ ii * 2 + 1 ] * cos( pos[ 2 ] ) + pos[ 1 ];

			fprintf( gp, "%lf %lf\n", x, y );
		}

		fprintf( gp, "\n\n" );
	}

public:
	GraphDrawer_RobotBase( void ) { }
	~GraphDrawer_RobotBase( void ) { }

	virtual void setConfig( config_property *c )
	{
		robot[ 0 ][ 0 ] = ( c->robot_info.length / 2 ) - 0.5;	robot[ 0 ][ 1 ] = c->robot_info.width / 2;
		robot[ 1 ][ 0 ] = ( c->robot_info.length / 2 );			robot[ 1 ][ 1 ] = 0.0;
		robot[ 2 ][ 0 ] = ( c->robot_info.length / 2 ) - 0.5;	robot[ 2 ][ 1 ] = -1.0 * ( c->robot_info.width / 2 );
		robot[ 3 ][ 0 ] = -1.0 * ( c->robot_info.length / 2 );	robot[ 3 ][ 1 ] = -1.0 * ( c->robot_info.width / 2 );
		robot[ 4 ][ 0 ] = -1.0 * ( c->robot_info.length / 2 );	robot[ 4 ][ 1 ] = c->robot_info.width / 2;
	}
	virtual void drawGraph( void ) = 0;
};

#endif
