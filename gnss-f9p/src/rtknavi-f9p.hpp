#ifndef _RTKNAVI_F9P_HPP
#define _RTKNAVI_F9P_HPP

#include <termios.h>
#include "gnss-f9p.hpp"

#define BAUDRATE	B460800	//B230400	//B115200	//B57600		//ボーレートの設定

// 受信機の通信に関するクラス
class RTKNAVI_F9P_BASE
{
protected:
	char device[ 256 ];	// デバイスファイル
	int fd; // シリアル通信ファイルディスクリプタ
	struct termios newtio;    // シリアル通信設定
	struct termios oldtio;

//	int readCommand_rmc( void );
	int readCommand_gga( void );
public:
	RTKNAVI_F9P_BASE( void ) { }
	~RTKNAVI_F9P_BASE( void ) { }
	
	void openGNSSReceiver( const char *dev );
	void closeGNSSReceiver( void );
	int readCommand( void );
	
	virtual bool decodeRTKGNSS( rtk_gnss_f9p *dst ) = 0;
};

//--- ベクトルの定義 
#define MAXN	12
typedef struct {
	int n;				// size of vector
	double a[ MAXN ];	// elements of vector 
	bool err; 			// err=true: error 
} my_vector_t;
//--- 行列の定義
typedef struct {
	int n;				// size of raw 
	int m; 				// size of column 
	double a[ MAXN ][ MAXN ];// elements of matrix 
	char message[ 80 ];	// error report 
	bool err; 			// err=true: error 
} matrix;

// 座標変換に関するクラス
// ( ECEF -> BLH )( BLH -> ECEF )( BLH, ECEF -> ENU )
class TransCoodinator
{
private:
	int coodinator_id;
	my_vector_t ecef_0;

public:
	TransCoodinator( void ){ }
	~TransCoodinator( void ){ }
	
	my_vector_t getECEF0( void )
	{
		return ecef_0;
	}
	void setCoodinatorID( int c );
	int getCoodinatorID( void )
	{
		return coodinator_id;
	}
	my_vector_t ecef2enu( my_vector_t dest, my_vector_t origin );
	my_vector_t ecef2blh( my_vector_t ec );
	my_vector_t blh2ecef( double phi, double ramda, double height );

};

//--- 芝刈り機用GNSSの本体
class RTKNAVI_F9P : public RTKNAVI_F9P_BASE
{
protected:
	TransCoodinator trans;
	
	bool decodeRTKGNSS_gga( rtk_gnss_f9p *dst );
//	bool decodeRTKGNSS_rmc( rtk_gnss_f9p *dst );
	
public:
	RTKNAVI_F9P( void ) { }
	~RTKNAVI_F9P( void ) { }
	
	void setCoodinatorID( int c ){ trans.setCoodinatorID( c ); }
	void printRTKGNSS( rtk_gnss_f9p *data );

	virtual bool decodeRTKGNSS( rtk_gnss_f9p *dst );
};

#endif // _RTKNAVI_F9P_HPP
