#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <ssm.hpp>
#include "imu.hpp"
#include "9axisIMU.hpp"
#include "utility.hpp"

#define CONV_GRAVITY	9.80665

void RT_IMU_Base::openIMU( const char *dev, unsigned int wait_time )
{
	sprintf( device, "%s", dev );
	printf( "Device = %s\n", device );

//	if( !( fd = open( device, O_RDWR ) ) ){		 // デバイスをオープンする
	if( !( fd = open( device, O_RDWR | O_NOCTTY) ) ){		 // デバイスをオープンする
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open RT-USB-9axisIMU\n" );
		exit( EXIT_FAILURE );
	}
	ioctl( fd, TCGETS, &oldtio );		// 現在のシリアルポートの設定を待避

	bzero( &newtio, sizeof( newtio ) );
	newtio = oldtio;					// ポートの設定をコピー

	cfmakeraw( &newtio );
	cfsetispeed( &newtio, BAUDRATE_IMU );
	cfsetospeed( &newtio, BAUDRATE_IMU );
	ioctl( fd, TCSETS, &newtio );		// ポートの設定を有効にする
	
	// 初期化（初期角度や角速度オフセットの推定）
	initialize( wait_time );
	property.K_Coef = 0.996;
}
void RT_IMU_Base::closeIMU( void )
{
	ioctl( fd, TCSETS, &oldtio );	// ポートの設定を元に戻す
	close( fd );					// デバイスのクローズ
}
void RT_IMU_Base::getIMUdata( imu_fs *data )
{
	data->status = imu.status;
	for( int i = 0 ; i < 3 ; i++ ){
		data->accel[ i ] = imu.accel[ i ];
		data->angvel[ i ] = imu.angvel[ i ] - property.angvel_offset[ i ];
		data->mag[ i ] = imu.mag[ i ];
		data->estAng[ i ] = imu.estAng[ i ];
	}
	data->temperature = imu.temperature;
}
void RT_IMU_Base::getIMUproperty( imu_property *data )
{
	data->gravity = property.gravity;
	for( int i = 0 ; i < 3 ; i++ ){
		data->angvel_offset[ i ] = property.angvel_offset[ i ];
	}
	for( int i = 0 ; i < 2 ; i++ ){
		data->initAng[ i ] = property.initAng[ i ];
	}
}
void RT_IMU_Base::initialize( unsigned int wait_time )
{
	Gprint( "Calculate offsets for AngularVelocity.\n" );
	fprintf( stderr, "wait %d seconds ... ", wait_time );
	// 初期化
	for( int i = 0 ; i < 3 ; i++ ){
		property.angvel_offset[ i ] = 0.0;
		imu.estAng[ i ] = 0.0;
	}
	// 変数の初期化
	double start_time = gettimeSSM( );
	int time_now = 0, time_cnt = 0;
	long int data_count = 0;
	property.initAng[ _ROLL ] = 0;	// 加速度から推定したロール角
	property.initAng[ _PITCH ] = 0;	// 加速度から推定したピッチ角
	property.gravity = 0;
	// オフセットの計算
	while( time_cnt <= wait_time ){
		time_now = ( int )( gettimeSSM( ) - start_time );
		// カウント更新
		if( time_now == time_cnt ){
			fprintf( stderr, "%d ", wait_time - time_now );
			time_cnt++;
		}
		// データ取得
		bool ret = receiveIMU( );
		if( !ret ) continue;
		data_count++;
		// 角速度オフセットのインクリメント
		for( int i = 0 ; i < 3 ; i++ ){
			property.angvel_offset[ i ] += imu.angvel[ i ];
		}
		// ロール・ピッチ角・重力加速度のインクリメント
		double roll_acc_tmp = atan( imu.accel[ _Y ] / imu.accel[ _Z ] );
		property.initAng[ _ROLL ] += roll_acc_tmp;
		double dist = sqrt( imu.accel[ _Y ]*imu.accel[ _Y ] + imu.accel[ _Z ]*imu.accel[ _Z ] );
		double pitch_acc_tmp = atan( imu.accel[ _X ] / dist );
		property.initAng[ _PITCH ] += pitch_acc_tmp;
		property.gravity += ( dist / cos( pitch_acc_tmp ) );
//		property.gravity -= ( imu.accel[ _Z ] / ( cos( pitch_acc_tmp ) * cos( roll_acc_tmp ) ) );
	}
	// 平均
	for( int i = 0 ; i < 3 ; i++ ){
		property.angvel_offset[ i ] /= data_count;
	}
	property.gravity /= ( double )data_count;
	property.initAng[ _ROLL ] /= ( double )data_count;
	property.initAng[ _PITCH ] /= ( double )data_count;
	fprintf( stderr, "... OK\n\n" );
	imu.estAng[ _ROLL ] = property.initAng[ _ROLL ];
	imu.estAng[ _PITCH ] = property.initAng[ _PITCH ];
	// 確認用
	Gprint( "Calibration #0\n" );
	printf( "Offset for Angular Velocity\n" );
	printf( "roll= %f, pitch= %f, yaw= %f[rad]\n", property.angvel_offset[ _ROLL ], property.angvel_offset[ _PITCH ], property.angvel_offset[ _YAW ] );
	printf( "Initial angles and Acceleration of Gravity\n" );
	printf( "roll= %f, pitch= %f[rad]\n", property.initAng[ _ROLL ], property.initAng[ _PITCH ] );
	printf( "Gravity = %f[m/s^2]\n", property.gravity );
	printf( "\n" );
}

static double Ang_acc[ 2 ];
void RT_IMU_Base::estAng4Acc( void )
{
	Ang_acc[ _ROLL ] = atan( imu.accel[ _Y ] / imu.accel[ _Z ] );
	double dist = sqrt( imu.accel[ _Y ]*imu.accel[ _Y ] + imu.accel[ _Z ]*imu.accel[ _Z ] );
	Ang_acc[ _PITCH ] = atan( imu.accel[ _X ] / dist );
}
void RT_IMU_Base::estAng4Angvel( double dt )
{
	double Ang_angv[ 3 ];
	double s1 = sin( imu.estAng[ _ROLL ] );
	double c1 = cos( imu.estAng[ _ROLL ] );
	double t1 = tan( imu.estAng[ _PITCH ] );
	double c2 = cos( imu.estAng[ _PITCH ] );
	
	double r = imu.angvel[ _ROLL ] - property.angvel_offset[ _ROLL ];
	double p = imu.angvel[ _PITCH ] - property.angvel_offset[ _PITCH ];
	double y = imu.angvel[ _YAW ] - property.angvel_offset[ _YAW ];
	
	Ang_angv[ _ROLL ] = r + ( p * s1 * t1 ) + ( y * c1 * t1 );
	Ang_angv[ _PITCH ] = ( p * c1 ) - ( y * s1 );
	Ang_angv[ _YAW ] = ( p * s1 / c2 ) + ( y * c1 / c2 );

	for( int i = 0 ; i < 3 ; i++ ){
		imu.estAng[ i ] = trans_q( imu.estAng[ i ] + Ang_angv[ i ] * dt );
	}
}
static double time_prev;
static bool flag_first_loop_cp = true;
bool RT_IMU_Base::ComplementaryFilter( void )
{
	if( flag_first_loop_cp ){
		time_prev = gettimeSSM( );
		flag_first_loop_cp = false;
		return false;
	} else {
		// 時間差分
		double time_diff = gettimeSSM( ) - time_prev;
		time_prev = gettimeSSM( );

		estAng4Acc( );
		estAng4Angvel( time_diff );
		imu.estAng[ _ROLL ] = property.K_Coef * imu.estAng[ _ROLL ] + ( 1.0 - property.K_Coef ) * Ang_acc[ _ROLL ];
		imu.estAng[ _PITCH ] = property.K_Coef * imu.estAng[ _PITCH ] + ( 1.0 - property.K_Coef ) * Ang_acc[ _PITCH ];
//		imu.estAng[ _YAW ] = imu.estAng[ _YAW ];
	}
	return true;
}

static char buff[ STRLEN ];
bool RT_9axisIMU::receiveIMU( void )
{
	int size = read( fd, buff, sizeof( buff ) - 1 );
//	printf( "[%d]%s",size,buff);

	if( size > 0  ){	// 受信データ

		if( size != 28 ) return false;

		for( int t = 0 ; t < size ; t += 2 ){
			int val = 0xff & buff[ t ] | ( ( 0xff&buff[ t+1 ] ) << 8 );
			if( val >= 32767 )	val -= 65536;

			if( t ==  8 ) imu.accel[ _X ] = ( double )val / 2048.0; // 要チェック
			if( t == 10 ) imu.accel[ _Y ] = ( double )val / 2048.0; // 要チェック
			if( t == 12 ) imu.accel[ _Z ] = ( double )val / 2048.0;
			
			if( t == 14 ) imu.temperature = ( double )val / 340.0 + 35.0;

			if( t == 16 ) imu.angvel[ _ROLL ] = DEG2RAD( ( double )val / 16.4 );
			if( t == 18 ) imu.angvel[ _PITCH ] = DEG2RAD( ( double )val / 16.4 );
			if( t == 20 ) imu.angvel[ _YAW ] = DEG2RAD( ( double )val / 16.4 );

			if( t == 22 ) imu.mag[ _X ] = ( double )val * 0.3;
			if( t == 24 ) imu.mag[ _Y ] = ( double )val * 0.3;
			if( t == 26 ) imu.mag[ _Z ] = ( double )val * 0.3;
			
		}
		imu.status = true;
		return true;
	}
	imu.status = false;
	return false;
}

#define STR_MAX 256
#define SCI_VMIN 100000 //[us]
#define SCI_TIMEOUT 5000*SCI_VMIN //[us]
#define rev_buffer_length	256
static char str[ rev_buffer_length ];
static bool flag_first_loop_rc = true;
int RT_9axisIMU_ACII::readCommand( void )
{
	int cnt = 0;
	int ret;
	int cnt_timeout = 0;
	bool update = true;

// LFの頭出し	
	while( flag_first_loop_rc ){
		char tmp[ rev_buffer_length ];
		ret = read( fd, &tmp, rev_buffer_length-1 );
//		fprintf( stderr, "size=%d, %s\n",ret, tmp );	// 確認用
		if( ret < 0 ){
			fprintf( stderr, "[readCommand]ret1=%d\n",ret );
			return -1;
		} else if( ret == 0 )
			cnt_timeout++;
		else {
			for( int i = 0 ; i < ret ; i++ ){
				if( tmp[ i ] == 0x0a ){
//					fprintf( stderr, "**[%d]size=%d, %s\n",i,ret, tmp );
					flag_first_loop_rc = false;
				}
			}
		}
		if( cnt >= STR_MAX ){
			fprintf(stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Receive too many data.\n" );
			return -1;
		}

//		if( cnt_timeout * SCI_VMIN > SCI_TIMEOUT ){
		if( cnt_timeout > SCI_TIMEOUT ){
			printf( "[%d/%d] ",cnt, rev_buffer_length );
//			printReadCommand( );
			fprintf(stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Time Out.\n" );
			return -1;
		}
//		return false;
	}
// IMUデータの読み込み
	while( update ){
		unsigned char tmp[ rev_buffer_length ];
		ret = read( fd, &tmp, rev_buffer_length-1 );
//		fprintf( stderr, "size=%d, %s\n",ret, tmp );
		if( ret < 0 ){
			fprintf( stderr, "[readCommand]ret1=%d\n",ret );
			return -1;
		} else if( ret == 0 )
			cnt_timeout++;
		else {
			for( int i = 0 ; i < ret ; i++ ){
				str[ cnt ] = tmp[ i ];
				if( tmp[ i ] == 0x0a ){
//					fprintf( stderr, "**[%d]size=%d, %s\n",i,ret, tmp );
					str[ cnt+1 ] = '\0';
					update = false;
					break;
				}
				cnt++;
			}
		}
		if( cnt >= STR_MAX ){
			fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Receive too many data.\n" );
			return -1;
		}

//		if( cnt_timeout * SCI_VMIN > SCI_TIMEOUT ){
		if( cnt_timeout > SCI_TIMEOUT ){
			printf( "[%d/%d] ",cnt, rev_buffer_length );
//			printReadCommand( );
			fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Time Out.\n" );
			return -1;
		}
	}

	return cnt;
}
bool RT_9axisIMU_ACII::receiveIMU( void )
{
	int size = readCommand( );
//	printf( "[%d]%s\n",size,str);

	if( size >= rev_buffer_length || size < 0 ){	// 受信データ
		if( size == -1 ){
			fprintf( stderr, "Read Failed : %s\n", device );
			exit( EXIT_FAILURE );
		}
		imu.status = false;
		return false;
		
	} else {
		int timestamp;
		int cnt = 0;
		int data_id = 1;
		char tmp[ 64 ];
		for( int i = 0 ; i <= size ; i++ ){
			if( ( str[ i ] != ',' ) && ( str[ i ] != 0x0a ) ){
				tmp[ cnt ] = str[ i ];
				cnt++;
			} else {
				tmp[ cnt ] = '\0';
				cnt = 0;
//				printf( "<%d>%s\n", i,tmp );	// 確認用
				switch( data_id ){
				case 1:
					timestamp = atoi( tmp );
					data_id++;
					break;
				case 2:
					imu.angvel[ _ROLL ] = atof( tmp );
					data_id++;
					break;
				case 3:
					imu.angvel[ _PITCH ] = atof( tmp );
					data_id++;
					break;
				case 4:
					imu.angvel[ _YAW ] = atof( tmp );
					data_id++;
					break;
				case 5:
					imu.accel[ _X ] = ( double )CONV_GRAVITY * atof( tmp );
					data_id++;
					break;
				case 6:
					imu.accel[ _Y ] = ( double )CONV_GRAVITY * atof( tmp );
					data_id++;
					break;
				case 7:
					imu.accel[ _Z ] = ( double )CONV_GRAVITY * atof( tmp );
					data_id++;
					break;
				case 8:
					imu.mag[ _X ] = atof( tmp );
					data_id++;
					break;
				case 9:
					imu.mag[ _Y ] = atof( tmp );
					data_id++;
					break;
				case 10:
					imu.mag[ _Z ] = atof( tmp );
					data_id++;
					break;
				case 11:
					imu.temperature = atof( tmp );
					data_id++;
					break;
				default:
					break;
				}

			}
		}
		imu.status = true;
		return true;
	}
	imu.status = false;
	return false;
}

void RT_9axisIMU_ACII_SIMPLE::estAng4Angvel( double dt )
{
//	printf( "simple\n" );
	for( int i = 0 ; i < 3 ; i++ ){
		imu.estAng[ i ] = trans_q( imu.estAng[ i ] + ( imu.angvel[ i ] - property.angvel_offset[ i ] ) * dt );
	}
}
