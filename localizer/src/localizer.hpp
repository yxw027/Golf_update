#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#include <ssmtype/spur-odometry.h>

#define LOCALIZER_SNAME "localizer"

typedef struct
{
	bool status;
	Spur_Odometry estPose;		// 推定したポーズ

	double imu_offset;
	double gnss_yaw;
	double gnss_vel[ 3 ];	// 0:_X, 1:_Y, 2:_V
	bool dir;		// 前後退のフラグ(前進:true, 後退:false)

} localizer;


#endif
