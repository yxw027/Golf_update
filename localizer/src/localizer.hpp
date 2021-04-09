#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#include <ssmtype/spur-odometry.h>
//#include "framework.hpp"

//+++++++ テストlocalizer ++++++
//#define TEST_LOCALIZER	// プログラム確認後に要削除

#define LOCALIZER_SNAME "localizer"

typedef struct
{
	bool status;
	Spur_Odometry estPose;		// 推定したポーズ

	double imu_offset;
	double gnss_yaw;
	double gnss_vel[ 3 ];	// 0:_X, 1:_Y, 2:_V

#ifdef TEST_LOCALIZER
	bool flag_slip;		// 一時的に、前後退のフラグに使用(前進:true, 後退:false)
	double smoothing_w;
#else
	bool dir;		// 前後退のフラグ(前進:true, 後退:false)
#endif

} localizer;


#endif
