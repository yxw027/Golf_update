 /*
  * Date : 2018.07.09
  * Update : 2019.01.12
  * Update : 2019.02.01
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_IMU_HPP_
#define _GRAPH_DRAWER_IMU_HPP_

#include <stdio.h>
#include <iostream>
#include <cstddef>
#include "log2txt.hpp"
#include "imu.hpp"
#include "GraphDrawer.hpp"

#define DATA_MAX 2000
#define TIME_WIDTH 15 // sec

class GraphDrawerIMU : public GraphDrawer
{
private:
	int graph_mode;
	bool flag_save;
	Log2Txt_IMU savefile;
	
	double curTime;
	double time[ DATA_MAX ];
	double gyro_ring[ DATA_MAX ][ 3 ];
	int gyro_ring_head;
	int gyro_ring_cnt;

	double accel_ring[ DATA_MAX ][ 3 ];
	int accel_ring_head;
	int accel_ring_cnt;		

	double mag_ring[ DATA_MAX ][ 3 ];
	int mag_ring_head;
	int mag_ring_cnt;
		
	double temperature_ring[ DATA_MAX ];
	int temperature_ring_head;
	int temperature_ring_cnt;	

	double est_ang_ring[ DATA_MAX ][ 3 ];
	int est_ang_ring_head;
	int est_ang_ring_cnt;

public:
	GraphDrawerIMU( void ) : flag_save( false ) { }
	~GraphDrawerIMU( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameters( int mode, bool flag );
	void printProperty( imu_property *data );
	void setIMU( imu_fs *imu, double t );
	virtual void drawGraph( void );
};

#endif


