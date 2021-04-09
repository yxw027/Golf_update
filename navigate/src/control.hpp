#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <ssmtype/spur-odometry.h>
//#include "chkVersion.hpp"

#define CONTROL_SNAME "control"

// 速度制御用変数
typedef struct
{
	double calStroke;	// 速度に対する、予め計算したストローク量
	double inpPos;		// モータードライバへ入力したストローク量（mm）
	double curPos;		// 現在のストローク量

	double d_vel;		// 目標速度
	double vel;			// 現在速度
} cntlVel;

// ライン追従制御用変数
typedef struct
{
	double distance;	// ラインに対する誤差
	double tgtTheta;		// 世界座標に対する目標方位
	double tgtAngVel;	// 目標角速度
	double tgtSteeringAng;	// 目標ステアリング角（ラジアン）
	double tgtHandleAng;	// 目標ハンドル角（Deg）

	double tgtHandleAngVel;	// 目標ハンドル角速度 (Deg/s)
	double tgtHandleAng_ex;		// 何秒後かの標ハンドル角（Deg）の予測	

	double inpAng;	// モータードライバへ入力した相対角度（度）
	double curAng;	// 現在の角度
	double curTorque;	// 現在トルク
} LineFollow;

// ハンドル角オフセットの推定
typedef struct
{
	double estSteeringAng;	// 現在速度と角速度から推定した推定ステアリング角
	double estHandleAng;	// 推定ステアリング角から推定した推定ハンドル角
	double steering_offset;	// ステアリング・オフセット
} estHandleOffset;

typedef struct
{
	LineFollow follow;
	estHandleOffset offset;
	cntlVel cVel;
} control;


#endif
