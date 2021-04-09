 /*
  * Date : 2019.08.21
  * Update : 2021.03.26
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_HANDLE_HPP_
#define _GRAPH_DRAWER_HANDLE_HPP_

#include <stdio.h>
#include <iostream>
#include <cstddef>
#include "OMcntl.hpp"
#include "GraphDrawer.hpp"
#include "log2txt.hpp"

#define DATA_MAX 2000
#define TIME_WIDTH 15 // sec

class GraphDrawerHandle : public GraphDrawer
{
private:
	bool flag_save;
	Log2Txt_OMcntl savefile;
		
	double curTime;
	double time[ DATA_MAX ];
	double handle_ring[ DATA_MAX ][ 7 ];
	int handle_ring_head;
	int handle_ring_cnt;

public:
	GraphDrawerHandle( void ) : handle_ring_head( 0 ), handle_ring_cnt( 0 ) { }
	~GraphDrawerHandle( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameters( int id, bool flag );
	void setHandleInfo( OMcntl *data, double t );
	virtual void drawGraph( void );
};

#endif


