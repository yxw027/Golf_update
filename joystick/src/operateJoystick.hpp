#ifndef _OPERATE_JOYSTICK_HPP
#define _OPERATE_JOYSTICK_HPP

#include <ssm.hpp>
#include <joystick.hpp>
#include "ModbusRTU.hpp"
#include "OMcntl.hpp"
#include "framework.hpp"

//	ssm::JS_BUTTON_0 -> SELECT
//	ssm::JS_BUTTON_3 -> START
//	ssm::JS_BUTTON_13 -> 丸ボタン
//	ssm::JS_BUTTON_14 -> ☓ボタン
//	ssm::JS_BUTTON_16 -> PS
//	0:左、左右(左-1、右1)
//	1:左、上下(上-1、下1)
//	2:右、左右(左-1、右1)
//	3:右、上下(上-1、下1)
class operateJoystick
{
protected:
	ModbusRTU_CLASS modbus;	// モータードライバと接続
	
	OMcntl motorInfo;
	unsigned long status_botton;
	config_property conf;
	
	void Free( void );
	void Start( void );
	void returnZero( void );
	void resetALM( void );
	void setZeroPosition( void );
	void Accel( ssm::JSData *joy2 );
	void Handle( ssm::JSData *joy2 );
	void Lever( int mode );
	bool returnNeutral_Lever( void );
	void getMotorInfo( int id );

public:
	operateJoystick( void ) : status_botton( 0 ) { }
	~operateJoystick( void ) { }

	void setConfig( config_property c )
	{
		conf = c;
		modbus.setConfig( c );
	}
	OMcntl getOMcntl( void )
	{
		return motorInfo;
	}
	void printProperty( ssm::JSProperty *joy );
	void openOMController( const char *dev );
	void closeOMController( void )
	{
		modbus.closeController( );
	}
	void operateBotton( ssm::JSData *joy1 );
	bool operateStick( ssm::JSData *joy1 );

};



#endif
