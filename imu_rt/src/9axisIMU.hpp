#ifndef _9AXIS_IMU_HPP
#define _9AXIS_IMU_HPP

#include <termios.h>
#include "imu.hpp"
#include "utility.hpp"

#define BAUDRATE_IMU	B57600		//B115200		//ボーレートの設定

class RT_IMU_Base
{
protected:
	char device[ STRLEN ];	// デバイスファイル
	
	int fd; // シリアル通信ファイルディスクリプタ
	struct termios newtio;    // シリアル通信設定
	struct termios oldtio;
	
	imu_fs imu;
	imu_property property;

	void initialize( unsigned int wait_time );	// 初期化（初期角度や角速度オフセットの推定）
	void estAng4Acc( void );	// 加速度から推定した角度(roll, pitch)
	virtual void estAng4Angvel( double dt );	// 角速度から推定した角度(roll, pitch, yaw)

public:
	RT_IMU_Base( void ){ }
	~RT_IMU_Base( void ){ }

	void openIMU( const char *dev, unsigned int wait_time );
	void closeIMU( void );

	void getIMUdata( imu_fs *data );
	void getIMUproperty( imu_property *data );
	bool ComplementaryFilter( void );	// 相補フィルター

	virtual bool receiveIMU( void ) = 0;
};

// 古いバージョンのIMU
class RT_9axisIMU : public RT_IMU_Base
{
public:
	RT_9axisIMU( void ){ }
	~RT_9axisIMU( void ){ }

	virtual bool receiveIMU( void );
	
};

// 新しいバージョンのIMU
class RT_9axisIMU_ACII : public RT_IMU_Base
{
public:
	RT_9axisIMU_ACII( void ) { }
	~RT_9axisIMU_ACII( void ) { }
	
	int readCommand( void );
	virtual bool receiveIMU( void );
};

// 角速度の座標変換せず、平面を移動しているとして角度を計算
class RT_9axisIMU_ACII_SIMPLE : public RT_9axisIMU_ACII
{
protected:
	virtual void estAng4Angvel( double dt );	// 角速度から推定した角度(roll, pitch, yaw)
public:
	RT_9axisIMU_ACII_SIMPLE( void ) { }
	~RT_9axisIMU_ACII_SIMPLE( void ) { }
	
};

#endif // AXISIMU_HPP
