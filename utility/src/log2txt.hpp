 /*
  * データをテキストファイルに保存するクラス
  * Date : 2021.03.26
  * Author : T.Hasegawa
  */
#ifndef _LOG_TO_TXT_HPP_
#define _LOG_TO_TXT_HPP_

#include <stdio.h>
#include <iostream>
#include <cstddef>
#include "imu.hpp"
#include "gnss-f9p.hpp"
#include "localizer.hpp"
#include "OMcntl.hpp"
#include "control.hpp"
#include "urg.hpp"

class Log2Txt_Base
{
protected:
	FILE *savefp;
	bool flag_first_loop;
	double start_time;
	
public:
	Log2Txt_Base( void ) : start_time( 0 ), flag_first_loop( true ) { }
	~Log2Txt_Base( void ){ }
	void openSaveFile( const char *save_filename );
	void closeSaveFile( void )
	{
		fclose( savefp );
	}
};
// ************************** for IMU ********************************
class Log2Txt_IMU : public Log2Txt_Base
{
public:
	Log2Txt_IMU( void ){ }
	~Log2Txt_IMU( void ){ }
	
	bool log2txt( imu_fs *imu, double t );
};
// ************************** for GNSS-F9P ********************************
class Log2Txt_GNSS_F9P : public Log2Txt_Base
{
public:
	Log2Txt_GNSS_F9P( void ){ }
	~Log2Txt_GNSS_F9P( void ){ }
	
	bool log2txt( rtk_gnss_f9p *data, double t );
};
// ************************** for Localizer ********************************
class Log2Txt_Localizer : public Log2Txt_Base
{
public:
	Log2Txt_Localizer( void ){ }
	~Log2Txt_Localizer( void ){ }
	
	bool log2txt( localizer *data, double t );
};
// ************************** for Motor ********************************
class Log2Txt_OMcntl : public Log2Txt_Base
{
private:
	int id;
public:
	Log2Txt_OMcntl( void ){ }
	~Log2Txt_OMcntl( void ){ }
	
	void setMotorID( int i );//{ id = i; }
	bool log2txt( OMcntl *data, double t );
};
// ************************** for Control ********************************
class Log2Txt_Control : public Log2Txt_Base
{
public:
	Log2Txt_Control( void ){ }
	~Log2Txt_Control( void ){ }
	
	bool log2txt( control *data, double t );
};
// ************************** for urg ********************************
class Log2Txt_URG : public Log2Txt_Base
{
public:
	Log2Txt_URG( void ){ }
	~Log2Txt_URG( void ){ }
	
	bool log2txt( urg_fs *data, double t );
};
#endif


