#ifndef ESTIMATE_POS_HPP
#define ESTIMATE_POS_HPP

#include <ssmtype/spur-odometry.h>
#include "utility.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "config.hpp"
#include "gnss-f9p.hpp"
#include "localizer.hpp"

class estimatePose
{
protected:
	// 推定したポーズ
	Spur_Odometry estPose;
	// config変数
	config_property cnf; 
	// センサ融合パラメータ
	double alpfa[ 3 ];	// for X, Y. Yaw
	double imu_offset;	// IMUで計算したYaw角を利用する場合に必要

	void setInitPose( Spur_Odometry p );
	void setInitPose( unsigned int wp_id );

public:
	estimatePose( void ) { }
	~estimatePose( void ) { }
	
	void printEstPose( void );
	void setParameters( const config_property c, bool flag, unsigned int wp_id );

	bool setIMUAngles( imu_fs *imu );
	virtual bool fusionEstPose( rtk_gnss_f9p *gnss, imu_fs *imu, localizer *est );
	
};

#endif
