 /*
  * Date : 2019.02.01
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_URG_HPP_
#define _GRAPH_DRAWER_URG_HPP_

#include <scip2awd.h>
#include "GraphDrawer.hpp"
#include "urg.hpp"

class GraphDrawerURG : public GraphDrawer
{
private:
	urg_fs scan;
	bool flag_intensity;
	bool flag_laser;
	
	void printProperty( urg_property *urg );

public:
	GraphDrawerURG( void ) { }
	~GraphDrawerURG( void ) { }

	void setParameter( bool laser, bool intensity, urg_property *urg )
	{
		flag_intensity = intensity;
		flag_laser = laser;
		printProperty( urg );
	}
	void setScan( urg_fs *urg );
	virtual void drawGraph( void );
};

#endif
