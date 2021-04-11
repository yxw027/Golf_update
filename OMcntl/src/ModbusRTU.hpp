#ifndef _MODBUS_RTU_HPP
#define _MODBUS_RTU_HPP

#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "OMcntl.hpp"
#include "config.hpp"

#define BAUDRATE_OM	B115200		//ボーレートの設定
//#define BAUDRATE_OM	B57600		//ボーレートの設定
//#define BAUDRATE_OM	B9600		//ボーレートの設定
#define _STR_MAX 128
#define SCI_VMIN 100000 //[us]
#define SCI_TIMEOUT 5000*SCI_VMIN //[us]

#define	_ACCEL_PRECISE		0.01
#define _HANDLE_PRECISE	0.01	// [degree/step]

// *****************************************************************
// オリエンタルモータ製モータドライバ AZD-KDの基本クラスModbusRTU_Base
// *****************************************************************
class ModbusRTU_Base
{
protected:
	config_property conf;

	char device[ _STR_MAX ];	// デバイスファイル
	unsigned char send_cmd[ _STR_MAX ];
	unsigned char read_cmd[ _STR_MAX ];
	unsigned char chk_cmd[ _STR_MAX ];
	int send_buffer_length;
	int rev_buffer_length;
	
	int fd; // シリアル通信ファイルディスクリプタ
	struct termios newtio;    // シリアル通信設定
	struct termios oldtio;
	
	void changeBaudrate( void );
	
	void setDeviceID( int id, int start );
	void setFunctionCode( int c, int start );
	void setRegAddr( int addr, int start );
	void setRegAddrNum( int addr_num, int start );
	void setByteNum( int bnum, int start );
	void setDriveData( int no, int start );
	void setDriveMethod( int met, int start );
	void setPos( int pos, int start );
	void setVel( int vel, int start );
	void setStartVelRate( int rate, int start );
	void setStopVelRate( int rate, int start );
	void setCurrentRate( int cur, int start );
	void setTrigger( int tr, int start );
	void setTransSite( int tr, int start );
	int setErrChk_send( int start );
	int setErrChk_chk( int start );
	
	void setMode( int no, int start );		// for sendStart, sendFree, and sendALM_RST, 
	void writeData( int data, int start );	// for returnZero, and resetZeroCommand
	void setChkCmd( void );
	
public:
	ModbusRTU_Base( void ) : send_buffer_length( 0 ), rev_buffer_length( 0 ) { }
	~ModbusRTU_Base( void ){ }
	void setConfig( const config_property c )
	{
		conf = c;
	}
	
	void openController( const char *dev );
	void closeController( void );
	bool sendCommand( void );
	bool readCommand( void );
	void printSendCommand( void );
	void printReadCommand( void );
	void printChkCommand( void );
	
	bool chkResponse( void );
	int Pos_mm2step( double p );
	int Ang_deg2step( double a );
	
	bool readRegistor( int id );	// ？？
	bool sendStart( int id );			// スタート
	bool sendFree( int id );			// フリー（スリー後はスタートを実行する必要あり。スタート後に動作開始）
	bool chkALM( int id );		// アラームの種類の確認
	bool sendALM_RST( int id );			// アラーム解除（解除できないアラームもある）
	bool returnZero( int id );			// 高速原点復帰→開始
	bool resetZeroCommand( int id );	// 高速原点復帰→終了
	bool setZeroPosition( int id );		// 原点の再設定
	
	bool clearPositionDeviation( int id );	// 位置偏差クリア
	bool resetPositionDeviation( int id );	// 位置偏差クリア後にモードのリセット

	virtual double getTorque( int id ) = 0;
	virtual double getPosition( int id ) = 0;
	virtual double getAngle( void ) = 0;
	virtual bool sendPos( int id, double pos ) = 0;	// リニアアクチュエータの位置制御（pos[cm]）
	virtual bool sendAng( int id, double ang ) = 0;	// 中空モータの相対角度制御（ang[deg]）
};
// *****************************************************************
// 直接参照方式用クラス
// *****************************************************************
class ModbusRTU_DirectOrder : public ModbusRTU_Base
{
protected:
	int getStep( int id );				// 現在ステップ数の獲得

public:
	ModbusRTU_DirectOrder( void ){ }
	~ModbusRTU_DirectOrder( void ){ }	

	virtual double getTorque( int id );
	virtual double getPosition( int id );
	virtual double getAngle( void );
	virtual bool sendPos( int id, double pos );	// リニアアクチュエータの位置制御（pos[cm]）
	virtual bool sendAng( int id, double ang );	// 中空モータの相対角度制御（ang[deg]）	
};
// *****************************************************************
// 間接参照方式用クラス
// *****************************************************************
class ModbusRTU_IndirectOrder : public ModbusRTU_Base
{
protected:
	OMcntl state;
	
	int getStep( void );			// 現在ステップ数の獲得
	int getTorque( void );			// 現在トルクと励磁最大静止トルクに対する割合で獲得
	int getAlarmCode( void );		// アラーム
	int getInfoCode( void );		// インフォメーション
	
public:
	ModbusRTU_IndirectOrder( void ) { }//state = {0}; }
	~ModbusRTU_IndirectOrder( void ) { }
	
	int getAlarmCode( int id );		// アラーム
	int getInfoCode( int id );		// インフォメーション
	void resetAngle( void );
	
	virtual double getTorque( int id );
	virtual double getPosition( int id );
	virtual double getAngle( void ){ return state.handle.ang; }
	virtual bool sendPos( int id, double pos );	// リニアアクチュエータの位置制御（pos[cm]）
	virtual bool sendAng( int id, double ang );	// 中空モータの相対角度制御（ang[deg]）

};

#endif
