#ifndef CONFIG_HPP
#define CONFIG_HPP

#define CONFIG_SNAME	"GM_Config" 
#define NUM_AREA_TYPE	2
#define NUM_GAIN_TYPE	2

#include <ssmtype/spur-odometry.h>
#include "utility.hpp"

typedef struct {
	int dummy;
} config;

typedef struct {
	double width;
	double length;
	double wheelbase;
} Robot_info;

typedef struct {
	int offset[ 3 ];	// offset_x[mm]		offset_y[mm]	offset_z[mm]
	int coordinate_id;	// 1系, 2系・・・・
} GNSS_info;

typedef struct {
	int rot[ 3 ];		// Roll		Pitch	Yaw [degree]
	int offset[ 3 ];	// offset_x[mm]		offset_y[mm]	offset_z[mm]
} URG_info;

typedef struct {
	double alpfa[ 3 ];	// for X, Y, Yaw
} Fusion_info;

typedef struct {
	char path[ STRLEN ];
	char filename[ STRLEN ];	// WP file
	double TR;	// Turning Radius 旋回半径[m]
} WP_info;

typedef struct {
	double fvel;	// [m/s]
	double bvel;	// [m/s]
	double ang_vel;	// [rad/s]
	double acc;		// [m/s^2]
	double ang_acc;	// [rad/s^2]
} Navi_info;

typedef struct {
	double x, y;
} Point;

typedef struct {
	Point p1, p2, p3, p4;
} Area;

typedef struct {
	double beta;	// ハンドル角オフセットの推定割合
	double s_ratio;	//(ハンドル角/ステアリング角)の比
// ハンドルの制御用ゲインの設定
	double l_K1[ NUM_GAIN_TYPE ];	// ライン追従制御用ゲイン//[deg/m]
	double l_K2[ NUM_GAIN_TYPE ];	// ライン追従制御用ゲイン//[1/s]
	double clip_tgtAngle;		// 目標侵入角度のクリップ角 [deg]
	double clip_tgtSteering;	// 目標ステアリング角のクリップ角 [deg]
// アクセルの制御用ゲインの設定
	double minStroke;	// [mm]
	double maxStroke;	// [mm]
	double v_K1;		// 速度制御用微分ゲイン	// 1[m/s]の差に対して何mm[ex.5mm]のストローク変化
	double v_K2;		// 速度制御用積分ゲイン	// 変化率１に対して、何mm[ex.0.5mm]のストローク
// カッティングユニット用レバーのニュートラルポジションへ戻すまでの時間
	unsigned int cuttingUnit_Down_time;	// 芝刈部を降ろす時間
	unsigned int cuttingUnit_Up_time;	// 芝刈部を上げる時間
} Control_info;

typedef struct {
// ハンドルを振切った際の設定
	double thrHandleTorque;	// ハンドルトルクの閾値 [0〜1000]
	double invHandleInput;	// 振切った際の逆回転入力（絶対値） [deg]
	double diffHandleAngle;	// 振切った際にハンドル角を戻す相対角度 [deg]
// 中空アクチュエータの設定（ハンドル）
	double h_angvel;	// [deg/s]
	double h_angacc;	// [deg/s^2]
// リニアアクチュエータの設定（アクセル）
	double a_vel;		// [mm/s]
	double a_acc;		// [mm/s^2]
} Motor_info;

typedef struct {
//****** for Robot info ******
	Robot_info robot_info;
//****** for GNSS Reciever *******
	GNSS_info gnss;
//****** Initial Position *******
	Spur_Odometry init_pose;
//****** for Fusion info *******
	Fusion_info fusion; 
//****** for Navigation *******
	Navi_info navi;
//****** for Way Point *******
	WP_info wp_info; 
//****** for CONTROL INFO *******
	Control_info cntl;
//****** for MOTER INFO *******
	Motor_info motor;
//****** for 3D URG ******
	URG_info urg3d;
//****** for Obstacle detection *******	
	Area dec_area[ NUM_AREA_TYPE ], stop_area[ NUM_AREA_TYPE ], avoid_area[ NUM_AREA_TYPE ];

} config_property;

#endif
