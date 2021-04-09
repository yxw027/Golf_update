 /*
  * Date : 2020.03.12
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_VELOCITY_HPP_
#define _GRAPH_DRAWER_VELOCITY_HPP_

#include <stdio.h>
#include <iostream>
#include <cstddef>
#include "localizer.hpp"
#include "GraphDrawer.hpp"
#include "log2txt.hpp"

#define DATA_MAX 2000
#define TIME_WIDTH 15 // sec

class GraphDrawerVel : public GraphDrawer
{
private:
	bool flag_save;
	Log2Txt_Localizer savefile;
		
	double curTime;
	double time[ DATA_MAX ];
	double vel_ring[ DATA_MAX ];
	int vel_ring_head;
	int vel_ring_cnt;
	
	void printPose( localizer *odm, double dt );

public:
	GraphDrawerVel( void ) : vel_ring_head( 0 ), vel_ring_cnt( 0 ) { }
	~GraphDrawerVel( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameters( bool flag );
	void setVel( localizer *odm, double t );
	virtual void drawGraph( void );
};

#endif


