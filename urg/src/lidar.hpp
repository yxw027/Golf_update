#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <scip2awd.h>
#include "urg.hpp"

class lidar_Hokuyo
{
protected:
	bool flag_intensity;
	// device valiant
	S2Sdd_t buf;		// URG multi buffer
	S2Scan_t *data;		// URG scan data buffer
	S2Param_t param;	// URG param info
	S2Ver_t ver;
	
public:
	lidar_Hokuyo( void ) : flag_intensity( true ) { }
	~lidar_Hokuyo( void ) { }
	
	bool initialize( char *dev, bool flag );
	bool getPointCloudData( urg_fs *data );
	void getProperty( urg_property *urg );
};

#endif
