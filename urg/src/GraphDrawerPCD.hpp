 /*
  * Date : 2020.10.03
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_PCD_HPP_
#define _GRAPH_DRAWER_PCD_HPP_

#include <stdio.h>
#include <iostream>
#include <cstddef>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "urg.hpp"

typedef pcl::PointXYZI PointType;

class GraphDrawerPCD
{
private:
	double mPos[ 3 ];		// LiDARの設置位置
	double mAng[ 3 ];		// LiDARの設置姿勢

	urg_fs scan;
	bool flag_intensity;
	bool flag_laser;
public:
	GraphDrawerPCD( void ) { }
	~GraphDrawerPCD( void ) { }

	void initialize( bool laser, bool intensity );
	void initLiDARPos( double x, double y, double z, double roll, double pitch, double yaw )
	{
		mPos[ 0 ] = x;
		mPos[ 1 ] = y;
		mPos[ 2 ] = z;
		mAng[ 0 ] = roll;
		mAng[ 1 ] = pitch;
		mAng[ 2 ] = yaw;
	}

	void setScan( urg_fs *urg );
	virtual bool drawGraph( void );
	
	void savePCD( void );
};

#endif


