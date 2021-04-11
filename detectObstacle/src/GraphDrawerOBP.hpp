 /*
  * Date : 2019.08.22
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_OBP_HPP_
#define _GRAPH_DRAWER_OBP_HPP_

#include "GraphDrawer.hpp"
#include "urg.hpp"
#include "detectObstacle.hpp"
#include "config.hpp"

class GraphDrawerOBP_2D : public GraphDrawer
{
protected:
	urg_fs scan;
	obp_fs obp;
	Area dec_area[ NUM_AREA_TYPE ], stop_area[ NUM_AREA_TYPE ], avoid_area[ NUM_AREA_TYPE ];
	
	bool flag_intensity;
	bool flag_laser;
	int area_type;
	config_property conf;

public:
	GraphDrawerOBP_2D( void ) { }
	~GraphDrawerOBP_2D( void ) { }

	void setParameter( bool laser, bool intensity, config_property *c );
	void setAreaType( int a )
	{
		area_type = a;
	}
	void setScan( urg_fs *urg );
	void setOBPoint( obp_fs *p );
	virtual void drawGraph( void );
};

class GraphDrawerOBP_3D : public GraphDrawerOBP_2D
{
public:
	GraphDrawerOBP_3D( void ) { }
	~GraphDrawerOBP_3D( void ) { }

	virtual void drawGraph( void );
};

#endif
