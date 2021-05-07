 /*
  * Date : 2019.02.01
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_URG_HPP_
#define _GRAPH_DRAWER_URG_HPP_

#include <scip2awd.h>
#include "GraphDrawer.hpp"
#include "urg.hpp"
#include "log2txt.hpp"

class GraphDrawerURG : public GraphDrawer
{
private:
	bool flag_save;
	Log2Txt_URG savefile;
	
	urg_fs scan;
	bool flag_intensity;
	bool flag_laser;
	
	void printProperty( urg_property *urg );

public:
	GraphDrawerURG( void ) { }
	~GraphDrawerURG( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameter( bool laser, bool intensity, urg_property *urg, bool sflag )
	{
		flag_intensity = intensity;
		flag_laser = laser;
		printProperty( urg );
		flag_save = sflag;
		if( flag_save ){
			savefile.openSaveFile( "log_urg.dat" );
		}
	}
	void setScan( urg_fs *urg, double t );
	virtual void drawGraph( void );
};

#endif
