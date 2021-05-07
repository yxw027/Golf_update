 /*
  * データをテキストファイルに保存するクラス
  * Date : 2021.03.26
  * Author : T.Hasegawa
  */
#include <stdlib.h>
#include "log2txt.hpp"
#include "utility.hpp"
#include "imu.hpp"
#include <math.h>

void Log2Txt_Base::openSaveFile( const char *save_filename )
{
	char filename[ STRLEN ];
	sprintf( filename, "../data/%s", save_filename );
	printf( "\nSave File = %s\n\n", filename );	// 確認用
	savefp = isValidFile( filename, "w", "dat" );
	flag_first_loop = true;
}
// ************************** for IMU ********************************
bool Log2Txt_IMU::log2txt( imu_fs *imu, double t )
{
	if( flag_first_loop ){
		start_time = t;
		flag_first_loop = false;
		return false;
	} else {
		fprintf( savefp, "%f ", t );	// 1
		fprintf( savefp, "%f ", t - start_time );	// 2
		fprintf( savefp, "%f %f %f ", imu->angvel[ _ROLL ], imu->angvel[ _PITCH ], imu->angvel[ _YAW ] );	// 3, 4, 5
		fprintf( savefp, "%f %f %f ", imu->accel[ _X ], imu->accel[ _Y ], imu->accel[ _Z ] );	// 6, 7, 8
		fprintf( savefp, "%f %f %f ", imu->mag[ _X ], imu->mag[ _Y ], imu->mag[ _Z ] );	// 9, 10, 11
		fprintf( savefp, "%f ", imu->temperature );	// 12, 13, 14
		fprintf( savefp, "%f %f %f\n", trans_q( imu->estAng[ _PITCH ] ), trans_q( imu->estAng[ _ROLL ] ), trans_q( imu->estAng[ _YAW ] ) );	// 15, 16, 17
	}
	return true;
}
// ************************** for GNSS-F9P ********************************
//static bool gnss_first_loop = true;
static double gnss_past[ 2 ] = { 0.0 };	// １ステップ前の測位点
static double gnss_time_past = 0;
bool Log2Txt_GNSS_F9P::log2txt( rtk_gnss_f9p *data, double t )
{
	if( data->status ){
		double vel[ 3 ] = { 0.0 };	// １ステップ前の測位点を使って計算した速度
		double yaw_gnss = 0;	// １ステップ前の測位点を使って計算したYaw角
		
		if( flag_first_loop ){	// 初期状態の処理
			
			start_time = t;
			
			gnss_past[ _X ] = data->enu.x;
			gnss_past[ _Y ] = data->enu.y;
			// 時間差分の計算のため１ステップ前の時間を保存
			gnss_time_past = t;
			flag_first_loop = false;
			return false;
			
		} else {	// 2回目以降の処理
			
			// 時間差計算
			double time_diff = t - gnss_time_past;
			if( time_diff > 0 ){
				double dx = data->enu.x - gnss_past[ _X ];
				double dy = data->enu.y - gnss_past[ _Y ];
				vel[ _X ] = dx / time_diff;    // x軸方向の速度ベクトルを計算
				vel[ _Y ] = dy / time_diff;    // y軸方向の速度ベクトルを計算
				vel[ _V ] = sqrt( vel[ _X ] * vel[ _X ] + vel[ _Y ] * vel[ _Y ] );    // 速度ベクトルのノルムを計算
				yaw_gnss = atan2( dy, dx );
			}
			// 速度用データ更新
			gnss_past[ _X ] = data->enu.x;
			gnss_past[ _Y ] = data->enu.y;
			gnss_time_past = t;
		}
			
		fprintf( savefp, "%f ", t );				// 1
		fprintf( savefp, "%f ", t - start_time );		// 2
		fprintf( savefp, "%f ", data->utc_time );		// 3
		fprintf( savefp, "%15.8f ", data->latitude );	// 4 (deg)
		fprintf( savefp, "%15.8f ", data->longitude );	// 5 (deg)
		fprintf( savefp, "%15.8f ", data->height );		// 6 (m)
		fprintf( savefp, "%d ", data->posStatus );		// 7 FIX or FLOAT

		fprintf( savefp, "%f ", data->ecef.x );	// 8
		fprintf( savefp, "%f ", data->ecef.y ); // 9
		fprintf( savefp, "%f ", data->ecef.z );	// 10
		fprintf( savefp, "%f ", data->enu.x );	// 11
		fprintf( savefp, "%f ", data->enu.y );	// 12
		fprintf( savefp, "%f ", data->enu.z );	// 13
		fprintf( savefp, "%f ", data->elevation );		//14 (m)
		fprintf( savefp, "%f ", data->Geoid_height );	//15 (m)
		
		fprintf( savefp, "%f ", vel[ _X ] );	// 16
		fprintf( savefp, "%f ", vel[ _Y ] );	// 17
		fprintf( savefp, "%f ", vel[ _V ] );	// 18
		fprintf( savefp, "%f ", yaw_gnss );// 19
		fprintf( savefp, "\n" );
	}
	return true;
}
// ************************** for Localizer ********************************
bool Log2Txt_Localizer::log2txt( localizer *data, double t )
{
	if( data->status ){
		if( flag_first_loop ){
			start_time = t;
			flag_first_loop = false;
			return false;
		} else {
			fprintf( savefp, "%f ", t );	// 1
			fprintf( savefp, "%f ", t - start_time );	// 2

			fprintf( savefp, "%f ", data->estPose.x );	// 3
			fprintf( savefp, "%f ", data->estPose.y );	// 4
			fprintf( savefp, "%f ", data->estPose.theta );	//5
		
			fprintf( savefp, "%f ", data->estPose.v );	// 6
			fprintf( savefp, "%f ", data->estPose.w );	// 7
			fprintf( savefp, "%d ", data->dir );		// 8
		
			fprintf( savefp, "%f ", data->imu_offset );	// 9
			fprintf( savefp, "%f ", data->gnss_yaw );	// 10

			fprintf( savefp, "%f ", data->gnss_vel[ _X ] );	// 11
			fprintf( savefp, "%f ", data->gnss_vel[ _Y ] );	// 12
			fprintf( savefp, "%f ", data->gnss_vel[ _V ] );	// 13

			fprintf( savefp, "\n" );
		}
	}
	return true;
}
// ************************** for Motor ********************************
void Log2Txt_OMcntl::setMotorID( int i )
{
	id = i;
	if( ( id != _ACCEL ) && ( id != _HANDLE ) && ( id != _LEVER ) ){
		fprintf( stderr, "Error! Log2Txt_OMcntl::setMotorID\n" );
		exit( EXIT_FAILURE );
	}
}
bool Log2Txt_OMcntl::log2txt( OMcntl *data, double time )
{
	if( flag_first_loop ){
		start_time = time;
		flag_first_loop = false;
		return false;
	} else {
		if( id == _ACCEL ){
			fprintf( savefp, "%f ", time );					// 1
			fprintf( savefp, "%f ", time - start_time );	// 2
			fprintf( savefp, "%f ", data->accel.input );	// 3
			fprintf( savefp, "%f ", data->accel.pos * 10 );	// 4
			fprintf( savefp, "%f ", data->accel.torque );	// 5
			fprintf( savefp, "%d ", data->accel.info );		// 6
			fprintf( savefp, "%d ", data->accel.alarm );	// 7
			fprintf( savefp, "\n" );
		} else if( id == _HANDLE ){
			fprintf( savefp, "%f ", time );					// 1
			fprintf( savefp, "%f ", time - start_time );	// 2	
			fprintf( savefp, "%f ", data->handle.input );	// 3
			fprintf( savefp, "%f ", data->handle.ang );	// 4
			fprintf( savefp, "%f ", data->handle.torque );// 5
			fprintf( savefp, "%d ", data->handle.info );	// 6
			fprintf( savefp, "%d ", data->handle.alarm );	// 7
			fprintf( savefp, "%f ", data->handle.angvel );// 8
			fprintf( savefp, "%f ", data->handle.target );// 9
			fprintf( savefp, "%f ", data->handle.target_angvel );// 10
			fprintf( savefp, "%f ", data->handle.target_ex );//11
			fprintf( savefp, "\n" );
		} else if( id == _LEVER ){
			fprintf( savefp, "%f ", time );	// 1
			fprintf( savefp, "%f ", time - start_time );	// 2
			fprintf( savefp, "%f ", data->lever.input );	// 3
			fprintf( savefp, "%f ", data->lever.pos );	// 4
			fprintf( savefp, "%f ", data->lever.torque );	// 5
			fprintf( savefp, "%d ", data->lever.info );	// 6
			fprintf( savefp, "%d ", data->lever.alarm );	//7
			fprintf( savefp, "\n" );		
		}
	}
	return true;
}
// ************************** for Control ********************************
bool Log2Txt_Control::log2txt( control *data, double t )
{
	if( flag_first_loop ){
		start_time = t;
		flag_first_loop = false;
		return false;
	} else {
		fprintf( savefp, "%f ", t );		// 1
		fprintf( savefp, "%f ", t - start_time  );	// 2
		// ライン追従制御用変数
		fprintf( savefp, "%f ", data->follow.distance );	// 3
		fprintf( savefp, "%f ", data->follow.tgtTheta );	// 4
		fprintf( savefp, "%f ", data->follow.tgtAngVel );	// 5

		fprintf( savefp, "%f ", data->follow.tgtSteeringAng );	// 6 目標ステアリング角（ラジアン）
		fprintf( savefp, "%f ", data->follow.tgtHandleAng );	// 7 目標ハンドル角（Deg）
		fprintf( savefp, "%f ", data->follow.inpAng );			// 8 モータードライバへ入力した相対角度（度）
		fprintf( savefp, "%f ", data->follow.curAng );			// 9 現在の角度
		fprintf( savefp, "%f ", data->follow.curTorque );		// 10 現在のトルク

		fprintf( savefp, "%f ", data->offset.estSteeringAng );	// 11 現在速度と角速度から推定したステアリング角
		fprintf( savefp, "%f ", data->offset.estHandleAng );	// 12 推定ステアリング角から推定したハンドル角
		fprintf( savefp, "%f ", data->offset.steering_offset );	// 13 ステアリング・オフセット

		// 速度制御用変数
		fprintf( savefp, "%f ", data->cVel.calStroke );		// 14 速度に対する、予め計算したストローク量
		fprintf( savefp, "%f ", data->cVel.inpPos );		// 15 モータードライバへ入力したストローク量（mm）
		fprintf( savefp, "%f ", data->cVel.curPos * 10.0 );	// 16 現在のストローク量（x10でmmへ変換）
		fprintf( savefp, "%f ", data->cVel.d_vel );			// 17 目標速度
		fprintf( savefp, "%f ", data->cVel.vel );			// 18 現在速度
		fprintf( savefp, "\n" );
	}
	return true;
}
// ************************** for urg ********************************
bool Log2Txt_URG::log2txt( urg_fs *data, double t )
{
	if( flag_first_loop ){
		start_time = t;
		flag_first_loop = false;
		return false;
	} else {
		fprintf( savefp, "%f ", t );	// 1
		fprintf( savefp, "%f ", t - start_time );	// 2
		fprintf( savefp, "%d ", data->size );	// 3
		for( int i = 0 ; i < data->size ; i++ ){
			fprintf( savefp, "%f ", data->length[ i ] );	// 4
			fprintf( savefp, "%f ", data->angle[ i ] );	// 5
			fprintf( savefp, "%d", data->intensity[ i ] );	// 6
		}
	}
	return true;
}
