 /*
  * Date : 2019.08.21
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_ACCEL_HPP_
#define _GRAPH_DRAWER_ACCEL_HPP_

#include <stdio.h>
#include <iostream>
#include <cstddef>
#include "OMcntl.hpp"
#include "GraphDrawer.hpp"
#include "log2txt.hpp"

#define DATA_MAX 2000
#define TIME_WIDTH 15 // sec

class GraphDrawerAccel : public GraphDrawer
{
private:
	bool flag_save;
	Log2Txt_OMcntl savefile;
		
	double curTime;
	double time[ DATA_MAX ];
	double accel_ring[ DATA_MAX ][ 3 ];
	int accel_ring_head;
	int accel_ring_cnt;

public:
	GraphDrawerAccel( void ) : accel_ring_head( 0 ), accel_ring_cnt( 0 ) { }
	~GraphDrawerAccel( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameters( int id, bool flag );
	void setAccelInfo( OMcntl *data, double t );
	virtual void drawGraph( void );
};

#endif


