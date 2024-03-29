#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include <ypspur.h>
#include <ssmtype/spur-odometry.h>
#include "imu.hpp"
#include "wp.hpp"
#include "gnss-f9p.hpp"
#include "config.hpp"
#include "localizer.hpp"
#include "utility.hpp"
#include "estimatePose.hpp"
#include "framework.hpp"

static double gnss_offset[ 3 ];	// 車両代表点への変換用変数
void estimatePose::setParameters( const config_property c, bool flag, unsigned int wp_id )
{
	cnf = c;

	if( flag ){
		printf( "Use the initial position calculated from WPfile\n" );
		setInitPose( wp_id );
	} else {
		printf( "Use the initial position written in GM_Config.cfg\n" );
		setInitPose( cnf.init_pose );
	}
	printEstPose( );	// 確認用
	// Alpfaの値
	alpfa[ _X ] = cnf.fusion.alpfa[ _X ];
	alpfa[ _Y ] = cnf.fusion.alpfa[ _Y ];
	alpfa[ _YAW ] = cnf.fusion.alpfa[ _YAW ];
	
	gnss_offset[ _X ] = ( double )cnf.gnss.offset[ _X ] / 1000.0;
	gnss_offset[ _Y ] = ( double )cnf.gnss.offset[ _Y ] / 1000.0;
	gnss_offset[ _Z ] = ( double )cnf.gnss.offset[ _Z ] / 1000.0;

}

bool estimatePose::setIMUAngles( imu_fs *imu )	// ジャイロオドメトリを使用しない場合はYaw角をセットするのみ
{
	// YAW角の更新
	estPose.theta = trans_q( imu->estAng[ _YAW ] + imu_offset );
	// 角速度の更新
	estPose.w = imu->angvel[ _YAW ];

	return true;
}
static bool flag_first_loop_fusionEstPose = true;
static double refPos[ 2 ] = { 0.0 };	// 車両代表点
static double refPos_b01m[ 2 ] = { 0.0 };	// 0.1m?前の車両代表点
static double refPos_b1step[ 2 ] = { 0.0 };	// １ステップ前の車両代表点
static double vel[ 3 ] = { 0.0 };	// １ステップ前の車両代表点を使って計算した速度
static double gnss_time_past = 0;
#ifdef Use_transReferencePoint_3D
static void transReferencePoint( rtk_gnss_f9p *pos, double roll, double pitch, double yaw );
#else
static void transReferencePoint( rtk_gnss_f9p *pos, double yaw );
#endif
static bool flag_VehicleDirection = true;//(前進:true, 後退:false)
bool estimatePose::fusionEstPose( rtk_gnss_f9p *gnss, imu_fs *imu, localizer *est )
{
	est->status = false;	// 推定位置が更新された場合、trueに変更する。

//	if( gnss->status && gnss->posStatus == RTK_FIX ){	// FIX時のみ計算
	if( gnss->status ){
		
		if( flag_first_loop_fusionEstPose ){	// 初期状態の処理
#ifdef Use_transReferencePoint_3D			
			// 車両代表点に変換
			transReferencePoint( gnss, imu->estAng[ _ROLL ], imu->estAng[ _PITCH ], estPose.theta );
#else
			// 車両代表点に変換
			transReferencePoint( gnss, estPose.theta );
#endif
			estPose.x = refPos[ _X ];
			estPose.y = refPos[ _Y ];
			estPose.v = 0;
			estPose.w = 0;

			refPos_b1step[ _X ] = estPose.x;
			refPos_b1step[ _Y ] = estPose.y;
			refPos_b01m[ _X ] = estPose.x;
			refPos_b01m[ _Y ] = estPose.y;

			// 時間差分の計算のため１ステップ前の時間を保存
			gnss_time_past = gnss->utc_time;
			flag_first_loop_fusionEstPose = false;
			
		} else {	// 2回目以降の処理
			// 時間差計算
			double time_diff = gnss->utc_time - gnss_time_past;

			if( time_diff > 0 ){
#ifdef Use_transReferencePoint_3D			
				// 車両代表点に変換
				transReferencePoint( gnss, imu->estAng[ _ROLL ], imu->estAng[ _PITCH ], estPose.theta );
#else
				// 車両代表点に変換
				transReferencePoint( gnss, estPose.theta );
#endif				
				estPose.x = refPos[ _X ];
				estPose.y = refPos[ _Y ];

				// 速度推定
				double dx = estPose.x - refPos_b1step[ _X ];
				double dy = estPose.y - refPos_b1step[ _Y ];
				vel[ _X ] = dx / time_diff;    // x軸方向の速度ベクトルを計算
				vel[ _Y ] = dy / time_diff;    // y軸方向の速度ベクトルを計算
				vel[ _V ] = sqrt( vel[ _X ] * vel[ _X ] + vel[ _Y ] * vel[ _Y ] );    // 速度ベクトルのノルムを計算
				estPose.v = vel[ _V ];
//				printf("%f\n",time_diff );
				
				// GNSSの方位推定(この後、IMUの方位と融合)	
				double dx01 = estPose.x - refPos_b01m[ _X ];
				double dy01 = estPose.y - refPos_b01m[ _Y ];
				double dL = sqrt( dx01 * dx01 + dy01 * dy01 );	// 移動距離の計算
				double estYaw_gnss = atan2( dy01, dx01 );

				// あまりにも移動速度が早い場合（突発誤差）は無視（エラー）
				if( vel[ _V ] > 5 ){	// 5[m/s]以上
					Rprint( "Error. " );	printf( "Over 5m/s\n" );
					return false;
				}

				// 30cm以上進んだ場合、Yaw角の融合
				if( ( dL > 0.3 ) ){
					
					// imu_offsetの更新	// 前進・後退の設定
					double gnss_ang_offset = trans_q( estYaw_gnss - imu->estAng[ _YAW ] - imu_offset );
					if( fabs( gnss_ang_offset ) > M_PI / 4 ){	// 後退時
						gnss_ang_offset = -SIGN( gnss_ang_offset ) * M_PI;
						flag_VehicleDirection = false;
					} else {	// 前進時
						gnss_ang_offset = 0;
						flag_VehicleDirection = true;
					}
					double yaw_diff = trans_q( estYaw_gnss - imu->estAng[ _YAW ] - imu_offset ) + gnss_ang_offset;
					// このif文は必要か？
					if( fabs( yaw_diff ) < M_PI / 4 ){		// GNSS方位とIMU方位間のオフセット再計算
						imu_offset += alpfa[ _YAW ] * yaw_diff;
					}
						
					// 方位用データ更新
					refPos_b01m[ _X ] = estPose.x;
					refPos_b01m[ _Y ] = estPose.y;
				}
				// 速度用データ更新
				refPos_b1step[ _X ] = estPose.x;
				refPos_b1step[ _Y ] = estPose.y;
				gnss_time_past = gnss->utc_time;
				
				// 推定位置（localizer）の更新
				est->status = true;
				est->dir = flag_VehicleDirection;
				est->estPose.x = estPose.x;
				est->estPose.y = estPose.y;
				est->estPose.theta = estPose.theta;
				est->estPose.v = estPose.v;
				est->estPose.w = estPose.w;
				est->imu_offset = imu_offset;
				est->gnss_yaw = estYaw_gnss;
				est->gnss_vel[ _X ] = vel[ _X ];
				est->gnss_vel[ _Y ] = vel[ _Y ];
				est->gnss_vel[ _V ] = vel[ _V ];

//				printEstPose( );	// 確認用
				
			} else {	// 同時刻のデータの場合はエラー
				return false;
			}
		}
		return true;
	}

	return false;
}
#ifdef Use_transReferencePoint_3D
static void transReferencePoint( rtk_gnss_f9p *pos, double roll, double pitch, double yaw )
{
//	printf("roll=%f, pitch=%f, yaw=%f\n", roll, pitch, yaw );
	double A[ 3 ][ 3 ];
	// 回転行列の計算
	A[ 0 ][ 0 ] = cos( yaw ) * cos( pitch );
	A[ 0 ][ 1 ] = -1.0 * sin( yaw ) * cos( roll ) + cos( yaw ) * sin( pitch ) * sin( roll );
	A[ 0 ][ 2 ] = sin( yaw ) * sin( roll ) + cos( yaw ) * sin( pitch ) * cos( roll );
	A[ 1 ][ 0 ] = sin( yaw ) * cos( pitch );
	A[ 1 ][ 1 ] = cos( yaw ) * cos( roll ) + sin( yaw ) * sin( pitch ) * sin( roll );
	A[ 1 ][ 2 ] = -1.0 * cos( yaw ) * sin( roll ) + sin( yaw ) * sin( pitch ) * cos( roll );
	A[ 2 ][ 0 ] = -1.0 * sin( pitch );
	A[ 2 ][ 1 ] = cos( pitch ) * sin( roll );
	A[ 2 ][ 2 ] = cos( pitch ) * cos( roll );
	// 代表点の位置へ座標変換
//	printf("gnss_offset[ _X ]=%f, gnss_offset[ _Y ]=%f, gnss_offset[ _Z ]=%f\n", gnss_offset[ _X ],gnss_offset[ _Y ],gnss_offset[ _Z ]);
	double dx = A[ 0 ][ 0 ] * gnss_offset[ _X ] + A[ 0 ][ 1 ] * gnss_offset[ _Y ] + A[ 0 ][ 2 ] * gnss_offset[ _Z ];
	double dy = A[ 1 ][ 0 ] * gnss_offset[ _X ] + A[ 1 ][ 1 ] * gnss_offset[ _Y ] + A[ 1 ][ 2 ] * gnss_offset[ _Z ];
//	double dz = A[ 2 ][ 0 ] * gnss_offset[ _X ] + A[ 2 ][ 1 ] * gnss_offset[ _Y ] + A[ 2 ][ 2 ] * gnss_offset[ _Z ];
//	printf("dx=%f, dy=%f\n", dx,dy);
	refPos[ _X ] = pos->enu.x + dx;
	refPos[ _Y ] = pos->enu.y + dy;
//	printf("refPos[ _X ]=%f, refPos[ _Y ]=%f\n", refPos[ _X ],refPos[ _Y ]);
}
#else
static void transReferencePoint( rtk_gnss_f9p *pos, double yaw )
{
	double A11 = cos( yaw );
	double A21 = sin( yaw );
	double A12 = -1.0 * A21;
	double A22 = A11;

	double gnss_offset_gl[ 2 ];
	gnss_offset_gl[ _X ] = A11 * gnss_offset[ _X ] + A12 * gnss_offset[ _Y ];
	gnss_offset_gl[ _Y ] = A21 * gnss_offset[ _X ] + A22 * gnss_offset[ _Y ];

	refPos[ _X ] = pos->enu.x + gnss_offset_gl[ _X ];
	refPos[ _Y ] = pos->enu.y + gnss_offset_gl[ _Y ];
}
#endif
void estimatePose::setInitPose( Spur_Odometry p )
{
	estPose = p;
	imu_offset = p.theta;// プログラム実行時のIMUのYaw角はゼロ度
//	printEstPose( );
}
#include "wp-mgr.hpp"
void estimatePose::setInitPose( unsigned int wp_id )
{
	WpMgr_Base wp_mgr;
	wp_mgr.initilize( cnf, wp_id, NAVI );

	wp_gl wp;
	wp = wp_mgr.getWPcurrent( );
	
	estPose.x = wp.x;
	estPose.y = wp.y;
	estPose.theta = wp.theta;
	estPose.v = 0;
	estPose.w = 0;
	imu_offset = estPose.theta;// プログラム実行時のIMUのYaw角はゼロ度	
}
void estimatePose::printEstPose( void )	// 確認用
{
	printf( "x=%f, y=%f, th=%f ", estPose.x, estPose.y, estPose.theta );
	printf( "v=%f, w=%f\n", estPose.v, estPose.w );
}
