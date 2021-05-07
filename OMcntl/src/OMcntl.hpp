#ifndef OM_CNTL_HPP
#define OM_CNTL_HPP

#define _ACCEL		1
#define _HANDLE		2
#define _LEVER		3

#define _ACCEL_POS_MIN		20
#define _ACCEL_POS_NEUTRAL	35
#define _ACCEL_POS_MAX		60

#define _LEVER_POS_MIN		52
#define _LEVER_POS_NEUTRAL	67
#define _LEVER_POS_MAX		85

#define OM_CNTL_SNAME	"OMcntl" 

typedef struct {
// 目標値
	double target;			// 目標値
	double target_angvel;	// 目標角速度
	double target_ex;		// 何秒後かの目標値の予測
// 現在値
	double input;	// 入力値
	double ang;		// 現在角度
	double angvel;	// 現在角速度
	double torque;	// 現在トルク
// エラー情報
	int info;		// インフォメーション
	int alarm;		// アラーム

} Handle_info;

typedef struct {
// 現在値
	double input;	// 入力値
	double pos;		// 現在位置
	double torque;	// 現在トルク
// エラー情報
	int info;		// インフォメーション
	int alarm;		// アラーム

} Accel_info;

typedef struct {
// 現在値
	double input;	// 入力値
	double pos;		// 現在位置
	double torque;	// 現在トルク
// エラー情報
	int info;		// インフォメーション
	int alarm;		// アラーム

} Lever_info;

typedef struct {
//****** for Steering ******
	Handle_info handle;	// [degree]
//****** for Accelerator ******
	Accel_info accel;
//****** for Lever *****
	Lever_info lever;

} OMcntl;

#endif
