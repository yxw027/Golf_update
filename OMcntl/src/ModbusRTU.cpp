#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include "ModbusRTU.hpp"

//#define NEW_VELOCITY_PROFILE

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) (x)
#else
#define DEBUG_PRINT(x)
#endif

// *****************************************************************
// オリエンタルモータ製モータドライバ AZD-KDの基本クラスModbusRTU_Base
// *****************************************************************
void ModbusRTU_Base::openController( const char *dev )
{
	sprintf( device, "%s", dev );
	DEBUG_PRINT( printf( "device = %s\n", device ) );
	
	if( !( fd = open( device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL ) ) ){		 // デバイスをオープンする
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open ModbusRTU\n" );
		exit( EXIT_FAILURE );
	}	
	changeBaudrate( );
}
void ModbusRTU_Base::closeController( void )
{
	tcsetattr( fd, TCSANOW, &oldtio );	// ポートの設定を元に戻す
	close( fd );					// デバイスのクローズ
}
void ModbusRTU_Base::changeBaudrate( void )
{
	int ret;

	tcgetattr( fd, &oldtio );		// 現在のシリアルポートの設定を待避
	memset( &newtio, 0, sizeof( newtio ) );	// 新しいポートの設定の構造体をクリアする

	ret = cfsetispeed( &newtio, BAUDRATE_OM );
	if( ret < 0 ){
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Failed to set input baud rate\n" );
		exit( EXIT_FAILURE );
	}
	ret = cfsetospeed( &newtio, BAUDRATE_OM );
	if( ret < 0 ){
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Failed to set output baud rate\n" );
		exit( EXIT_FAILURE );
	}

	/*  C_CFLAG      Control options
		CLOCAL       Local line - do not change "owner" of port
		CREAD        Enable receiver
	*/
	newtio.c_cflag |= ( CREAD | CLOCAL );
	/* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

	/*  Set data bits (5, 6, 7, 8 bits)
		CSIZE        Bit mask for data bits
	*/
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;

	/* Stop bit (1 or 2) */
	//	1
	newtio.c_cflag &=~ CSTOPB;
	//	else /* 2 */
//	newtio.c_cflag |= CSTOPB;

	/*  PARENB       Enable parity bit
		PARODD       Use odd parity instead of even */
//	if (ctx_rtu->parity == 'N') {
		/* None */
//		newtio.c_cflag &=~ PARENB;
//	} else if (ctx_rtu->parity == 'E') {
		/* Even */
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &=~ PARODD;
//	} else {
		/* Odd */
//		newtio.c_cflag |= PARENB;
//		newtio.c_cflag |= PARODD;
//	}

	/* Read the man page of termios if you need more information. */

	/*  This field isn't used on POSIX systems
		tios.c_line = 0;
	*/

	/* C_LFLAG      Line options

	ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
	ICANON       Enable canonical input (else raw)
	XCASE        Map uppercase \lowercase (obsolete)
	ECHO Enable echoing of input characters
	ECHOE        Echo erase character as BS-SP-BS
	ECHOK        Echo NL after kill character
	ECHONL       Echo NL
	NOFLSH       Disable flushing of input buffers after
	interrupt or quit characters
	IEXTEN       Enable extended functions
	ECHOCTL      Echo control characters as ^char and delete as ~?
	ECHOPRT      Echo erased character as character erased
	ECHOKE       BS-SP-BS entire line on line kill
	FLUSHO       Output being flushed
	PENDIN       Retype pending input at next read or input char
	TOSTOP       Send SIGTTOU for background output

	Canonical input is line-oriented. Input characters are put
	into a buffer which can be edited interactively by the user
	until a CR (carriage return) or LF (line feed) character is
	received.

	Raw input is unprocessed. Input characters are passed
	through exactly as they are received, when they are
	received. Generally you'll deselect the ICANON, ECHO,
	ECHOE, and ISIG options when using raw input
	*/

	/* Raw input */
	newtio.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

	/* C_IFLAG      Input options

		Constant     Description
		INPCK        Enable parity check
		IGNPAR       Ignore parity errors
		PARMRK       Mark parity errors
		ISTRIP       Strip parity bits
		IXON Enable software flow control (outgoing)
		IXOFF        Enable software flow control (incoming)
		IXANY        Allow any character to start flow again
		IGNBRK       Ignore break condition
		BRKINT       Send a SIGINT when a break condition is detected
		INLCR        Map NL to CR
		IGNCR        Ignore CR
		ICRNL        Map CR to NL
		IUCLC        Map uppercase to lowercase
		IMAXBEL      Echo BEL on input line too long
	*/
//	if (ctx_rtu->parity == 'N') {
		/* None */
//		newtio.c_iflag &= ~INPCK;
//	} else {
		newtio.c_iflag |= INPCK;
//	}
	
	/* Software flow control is disabled */
	newtio.c_iflag &= ~( IXON | IXOFF | IXANY );

	/* C_OFLAG      Output options
		OPOST        Postprocess output (not set = raw output)
		ONLCR        Map NL to CR-NL

		ONCLR ant others needs OPOST to be enabled
	*/

	/* Raw ouput */
	newtio.c_oflag &= ~OPOST;

	/* C_CC         Control characters
		VMIN         Minimum number of characters to read
		VTIME        Time to wait for data (tenths of seconds)

		UNIX serial interface drivers provide the ability to
		specify character and packet timeouts. Two elements of the
		c_cc array are used for timeouts: VMIN and VTIME. Timeouts
		are ignored in canonical input mode or when the NDELAY
		option is set on the file via open or fcntl.

		VMIN specifies the minimum number of characters to read. If
		it is set to 0, then the VTIME value specifies the time to
		wait for every character read. Note that this does not mean
		that a read call for N bytes will wait for N characters to
		come in. Rather, the timeout will apply to the first
		character and the read call will return the number of
		characters immediately available (up to the number you
		request).

		If VMIN is non-zero, VTIME specifies the time to wait for
		the first character read. If a character is read within the
		time given, any read will block (wait) until all VMIN
		characters are read. That is, once the first character is
		read, the serial interface driver expects to receive an
		entire packet of characters (VMIN bytes total). If no
		character is read within the time allowed, then the call to
		read returns 0. This method allows you to tell the serial
		driver you need exactly N bytes and any read call will
		return 0 or N bytes. However, the timeout only applies to
		the first character read, so if for some reason the driver
		misses one character inside the N byte packet then the read
		call could block forever waiting for additional input
		characters.

		VTIME specifies the amount of time to wait for incoming
		characters in tenths of seconds. If VTIME is set to 0 (the
		default), reads will block (wait) indefinitely unless the
		NDELAY option is set on the port with open or fcntl.
	*/
	/* Unused because we use open with the NDELAY option */
	newtio.c_cc[ VMIN ] = 0;
	newtio.c_cc[ VTIME ] = 0;

	if( tcsetattr( fd, TCSANOW, &newtio ) < 0 ){
		closeController( );
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Failed to set attribute of serial port\n" );
		exit( EXIT_FAILURE );
	}

	{ // ---> check bit rate
		struct termios term;
		speed_t isp, osp;

		ret = tcgetattr( fd, &term );
		if( ret < 0 ){
			fprintf(stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Failed to get attribute of serial port\n");
			exit( EXIT_FAILURE );
		}
		isp = cfgetispeed( &term );
		osp = cfgetospeed( &term );

		if( isp != BAUDRATE_OM || osp != BAUDRATE_OM ){
			// fail to set bit rate
			fprintf(stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Baudrate did not changed.\n");
			exit( EXIT_FAILURE );
		}
	} // <--- check bit rate

}
void ModbusRTU_Base::setChkCmd( void )
{
	for( int i = 0 ; i < rev_buffer_length - 2 ; i++ ){
		chk_cmd[ i ] = send_cmd[ i ];
	}
	setErrChk_chk( rev_buffer_length - 2 );		// エラーチェック

}

bool ModbusRTU_Base::readRegistor( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x03, 1 );// ファンクションコード=0003h 保持レジスタの読み出し
	setRegAddr( 0x1840, 2 );	// 保持レジスタ先頭アドレス=1840h
	setRegAddrNum( 6, 4 );		// 保持レジスタから呼び出す数=6個
	setErrChk_send( 6 );			// エラーチェック

	send_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	return true;
}

void ModbusRTU_Base::setMode( int no, int start )
{
//	int start = 7;
	
	int lo_2 = no & 0x000000FF;
	int lo_1_tmp = no & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = no & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = no & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
// 位置偏差クリア
bool ModbusRTU_Base::clearPositionDeviation( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x007C, 2 );	// 保持レジスタ先頭アドレス=007Ch
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000001, 7 );	// CLR（位置偏差クリア）
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );	// 1ms
	
	return true;
}
// 位置偏差クリア後にモードのリセット
bool ModbusRTU_Base::resetPositionDeviation( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x007C, 2 );	// 保持レジスタ先頭アドレス=007Ch
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000000, 7 );	// CLR（位置偏差クリア）のリセット
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );	// 1ms
	
	return true;
}
bool ModbusRTU_Base::sendStart( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x007C, 2 );	// 保持レジスタ先頭アドレス=007Ch
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000008, 7 );	// START
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );	// 1ms
	
	return true;
}
bool ModbusRTU_Base::sendFree( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x007C, 2 );	// 保持レジスタ先頭アドレス=007Ch
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000040, 7 );	// FREE
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );	// 1ms

	return true;
}
bool ModbusRTU_Base::chkALM( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x03, 1 );// ファンクションコード=0003h 保持レジスタの読み出し
	setRegAddr( 0x0080, 2 );	// 保持レジスタ先頭アドレス=00CCh
	setRegAddrNum( 2, 4 );		// 保持レジスタから呼び出す数=2個
	setErrChk_send( 6 );			// エラーチェック

	send_buffer_length = 8;
//	rev_buffer_length = 9;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = readCommand( );
		printReadCommand( );
		usleep( 1 * 1000 );	//1ms
	}
	usleep( 1 * 1000 );	// 1ms

	return true;
}
bool ModbusRTU_Base::sendALM_RST( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x007C, 2 );	// 保持レジスタ先頭アドレス=007Ch
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000080, 7 );	// ALM-RST
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );	// 1ms
					
	return true;
}
bool ModbusRTU_Base::setZeroPosition( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x018A, 2 );	// 保持レジスタ先頭アドレス=018Ah
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000000, 7 );	// ??
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
//		printReadCommand( );
		usleep( 10 * 1000 );	// 10ms
	}
// ポジション決定	
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x018A, 2 );	// 保持レジスタ先頭アドレス=018Ah
	setRegAddrNum( 2, 4 );		// 書き込みレジスタ数=2個
	setByteNum( 4, 6 );		// 書き込みバイト数=4 Byte
	setMode( 0x00000001, 7 );	// ??
	setErrChk_send( 11 );		// エラーチェック
	
	send_buffer_length = 13;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );	// 1ms
	ret = false;
	while( !ret ){
		ret = chkResponse( );
//		printReadCommand( );
		usleep( 10 * 1000 );	// 10ms
	}
	usleep( 1 * 1000 );	// 1ms
	
	return true;
}
bool ModbusRTU_Base::sendCommand( void )
{
	int cnt = 0;
	int ret;

	while( cnt < send_buffer_length ){
		if( ( ret = write( fd, &send_cmd[ cnt ], send_buffer_length - cnt) ) < 0 ){
			DEBUG_PRINT( printf( "[sendCommand]ret1=%d\n",ret ) );
			return false;
		}
		cnt += ret;
    }
	DEBUG_PRINT( printf( "[sendCommand]ret2=%d\n",ret ) );
	return true;
}

bool ModbusRTU_Base::readCommand( void )
{
	int cnt = 0;
	int ret;
	int cnt_timeout = 0;
	int len = rev_buffer_length;
//printf("ccc\n");
	DEBUG_PRINT( printf( "[read] " ) );
	while( cnt < len ){
		ret = read( fd, &read_cmd[ cnt ], len - cnt );
		if( ret < 0 ){
			DEBUG_PRINT( printf( "[readCommand]ret1=%d\n",ret ) );
			return false;
		} else if( ret == 0 )
			cnt_timeout++;
		else
			cnt += ret;

		if( cnt >= STR_MAX ){
			DEBUG_PRINT( fprintf(stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Receive too many data.\n" ) );
			return false;
		}

//		if( cnt_timeout * SCI_VMIN > SCI_TIMEOUT ){
		if( cnt_timeout > SCI_TIMEOUT ){
			printf( "[%d/%d] ",cnt, len );
			printReadCommand( );
			DEBUG_PRINT( fprintf(stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Time Out.\n" ) );
			return false;
		}

	}

//	sprintf( read_cmd, "%s", str );
//	DEBUG_PRINT( printf( "ret=%d\n", ret ) );
	DEBUG_PRINT( printReadCommand( ) );
	return true;
}
void ModbusRTU_Base::writeData( int data, int start )
{
//	int start = 4;
	int lo = data & 0x00FF;
	int hi_tmp = data & 0xFF00;
	int hi = hi_tmp >> 8;
	send_cmd[ start ] = hi;
	send_cmd[ start + 1 ] = lo;
}
bool ModbusRTU_Base::returnZero( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x06, 1 );// ファンクションコード=0006h
	setRegAddr( 0x007D, 2 );	// 保持レジスタ先頭アドレス=007Dh
	writeData( 0x0010, 4 );	// 高速原点復帰（ZHOMEをON）
	setErrChk_send( 6 );		// エラーチェック
	
	send_buffer_length = 8;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );
	
	return true;
}
bool ModbusRTU_Base::resetZeroCommand( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x06, 1 );// ファンクションコード=0006h
	setRegAddr( 0x007D, 2 );	// 保持レジスタ先頭アドレス=007Dh
	writeData( 0x0000, 4 );	// 高速原点復帰モード終了（ZHOMEをOFF）
	setErrChk_send( 6 );		// エラーチェック
	
	send_buffer_length = 8;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 1 * 1000 );
	bool ret = false;
	while( !ret ){
		ret = chkResponse( );
		usleep( 1 * 1000 );
	}
	usleep( 1 * 1000 );
	
	return true;
}
bool ModbusRTU_Base::chkResponse( void )
{
//	printReadCommand( );
//	printSendCommand( );
	readCommand( );
	for( int i = 0 ; i < rev_buffer_length ; i++ ){
		if( chk_cmd[ i ] != read_cmd[ i ] ) return false;
	}
	return true;
}
int ModbusRTU_Base::Pos_mm2step( double p )
{
/*
 	double lead = 6;	// 6mm
	double rot = p / lead;
	int step = 600;	// 1周600step
	int step_cnt = ( int )( ( double )step * rot );
*/
	int step_cnt = ( int )( p * 100.0 );
	return step_cnt;
}
int ModbusRTU_Base::Ang_deg2step( double a )
{
	int step_cnt = ( int )( a / ( double )_HANDLE_PRECISE );
//	printf("ang=%f, step=%d\n",a, step_cnt);
	return step_cnt;
}
void ModbusRTU_Base::setDeviceID( int id, int start )
{
	send_cmd[ start ] = id;
}

void ModbusRTU_Base::setFunctionCode( int c, int start )
{
	send_cmd[ start ] = c;
}

void ModbusRTU_Base::setRegAddr( int addr, int start )
{
	int lo = addr & 0x00FF;
	int hi_tmp = addr & 0xFF00;
	int hi = hi_tmp >> 8;
	send_cmd[ start ] = hi;
	send_cmd[ start + 1 ] = lo;
}
void ModbusRTU_Base::setRegAddrNum( int addr_num, int start )
{
	rev_buffer_length = 2 * addr_num + 5;
	
	int lo = addr_num & 0x00FF;
	int hi_tmp = addr_num & 0xFF00;
	int hi = hi_tmp >> 8;
	send_cmd[ start ] = hi;
	send_cmd[ start + 1 ] = lo;
}

void ModbusRTU_Base::setByteNum( int bnum, int start )
{
	send_cmd[ start ] = bnum;
}
void ModbusRTU_Base::setDriveData( int no, int start )
{
	int lo_2 = no & 0x000000FF;
	int lo_1_tmp = no & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = no & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = no & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setDriveMethod( int met, int start )
{
	int lo_2 = met & 0x000000FF;
	int lo_1_tmp = met & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = met & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = met & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setPos( int pos, int start )
{
	int lo_2 = pos & 0x000000FF;
	int lo_1_tmp = pos & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = pos & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = pos & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setVel( int vel, int start )
{
	int lo_2 = vel & 0x000000FF;
	int lo_1_tmp = vel & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = vel & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = vel & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setStartVelRate( int rate, int start )
{
	int lo_2 = rate & 0x000000FF;
	int lo_1_tmp = rate & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = rate & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = rate & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setStopVelRate( int rate, int start )
{
	int lo_2 = rate & 0x000000FF;
	int lo_1_tmp = rate & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = rate & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = rate & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setCurrentRate( int cur, int start )
{
	int lo_2 = cur & 0x000000FF;
	int lo_1_tmp = cur & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = cur & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = cur & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setTrigger( int tr, int start )
{
	int lo_2 = tr & 0x000000FF;
	int lo_1_tmp = tr & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = tr & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = tr & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
void ModbusRTU_Base::setTransSite( int tr, int start )
{
	int lo_2 = tr & 0x000000FF;
	int lo_1_tmp = tr & 0x0000FF00;
	int lo_1 = lo_1_tmp >> 8;
	
	int hi_2_tmp = tr & 0x00FF0000;
	int hi_2 = hi_2_tmp >> 16;
	int hi_1_tmp = tr & 0xFF000000;
	int hi_1 = hi_1_tmp >> 24;
	
	send_cmd[ start ] = hi_1;
	send_cmd[ start + 1 ] = hi_2;
	send_cmd[ start + 2 ] = lo_1;
	send_cmd[ start + 3 ] = lo_2;
}
// Table of CRC values for high-order byte 
static const int table_crc_hi[ ] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
// Table of CRC values for low-order byte 
static const int table_crc_lo[ ] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
int ModbusRTU_Base::setErrChk_send( int start )
{
	int crc_hi = 0xFF;	// high CRC byte initialized 
	int crc_lo = 0xFF;	// low CRC byte initialized 
	unsigned int i;	// will index into CRC lookup 
	int buffer_length = start;
	unsigned char *buffer = send_cmd;
	
	// pass through message buffer 
	while( buffer_length-- ){
		i = crc_hi ^ *buffer++;	// calculate the CRC  
		crc_hi = crc_lo ^ table_crc_hi[ i ];
		crc_lo = table_crc_lo[ i ];
	}

	send_cmd[ start ] = ( char )crc_hi;
	send_cmd[ start + 1 ] = ( char )crc_lo;
	return( crc_hi << 8 | crc_lo );
}

int ModbusRTU_Base::setErrChk_chk( int start )
{
	int crc_hi = 0xFF;	// high CRC byte initialized 
	int crc_lo = 0xFF;	// low CRC byte initialized 
	unsigned int i;	// will index into CRC lookup 
	int buffer_length = start;
	unsigned char *buffer = chk_cmd;
	
	// pass through message buffer 
	while( buffer_length-- ){
		i = crc_hi ^ *buffer++;	// calculate the CRC  
		crc_hi = crc_lo ^ table_crc_hi[ i ];
		crc_lo = table_crc_lo[ i ];
	}

	chk_cmd[ start ] = ( char )crc_hi;
	chk_cmd[ start + 1 ] = ( char )crc_lo;
	return( crc_hi << 8 | crc_lo );
}
void ModbusRTU_Base::printSendCommand( void )
{
	for( int i = 0 ; i < send_buffer_length ; i++ )
		printf( "%02X ", send_cmd[ i ] );
	printf( "\n" );
}
void ModbusRTU_Base::printReadCommand( void )
{
	for( int i = 0 ; i < rev_buffer_length ; i++ )
		printf( "%02X ", read_cmd[ i ] );
	printf( "\n" );
}
void ModbusRTU_Base::printChkCommand( void )
{
	for( int i = 0 ; i < rev_buffer_length ; i++ )
		printf( "%02X ", chk_cmd[ i ] );
	printf( "\n" );
}
// *****************************************************************
// 直接参照方式用クラス
// *****************************************************************
bool ModbusRTU_DirectOrder::sendPos( int id, double pos )
{
	if( pos >= 100 ) pos = 100;
	if( pos <= 0   ) pos = 0;
//	printf("pos=%f\n",pos);

	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x0058, 2 );	// 書き込みレジスタ先頭アドレス=0058h
	setRegAddrNum( 16, 4 );	// 書き込みレジスタ数=16個
	setByteNum( 32, 6 );		// 書き込みバイト数=32 Byte
	setDriveData( 0, 7 );		// 運転データNo.=0
	setDriveMethod( 1, 11 );	// 方式=1:絶対位置決め
//	setDriveMethod( 2, 11 );	// 方式=2:相対位置決め(指令位置基準)
	setPos( Pos_mm2step( pos ), 15 );			// 位置=pos[mm]//8,500 step
	
	setVel( Pos_mm2step( conf.motor.a_vel ), 23 );			// 速度=200[mm/s]
	setStartVelRate( Pos_mm2step( conf.motor.a_acc ), 27 );	// 起動・変速レート=200[mm/s^2]
	setStopVelRate( Pos_mm2step( conf.motor.a_acc ), 31 );	// 停止レート=200[mm/s^2]

	setCurrentRate( 0x000003E8, 31 );// 運転電流=100.0 %
	setTrigger( 1, 35 );		// 反映トリガ=1:全データ反映
	setErrChk_send( 39 );		// エラーチェック
	
	send_buffer_length = 41;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );

	usleep( 2 * 1000 );	//2ms
	bool ret = false;
	while( !ret ){
		usleep( 1 * 1000 );
		ret = chkResponse( );
	}
	usleep( 2 * 1000 );
	
	return true;
}
bool ModbusRTU_DirectOrder::sendAng( int id, double ang )
{
	if( ang >= 180.0 ) ang = 179.0;
	if( ang <= -180.0 ) ang = -179.0;

	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x10, 1 );// ファンクションコード=0010h
	setRegAddr( 0x0058, 2 );	// 書き込みレジスタ先頭アドレス=0058h
	setRegAddrNum( 16, 4 );	// 書き込みレジスタ数=16個
	setByteNum( 32, 6 );		// 書き込みバイト数=32 Byte
	setDriveData( 0, 7 );		// 運転データNo.=0
//	setDriveMethod( 1, 11 );	// 方式=1:絶対位置決め
//	setDriveMethod( 2, 11 );	// 方式=2:相対位置決め(指令位置基準)
	setDriveMethod( 3, 11 );	// 方式=2:相対位置決め(検出位置基準)
//	setDriveMethod( 22, 11 );	// 方式=2:相対位置決め押し当て(検出位置基準)
	setPos( Ang_deg2step( ang ), 15 );			// 角度=ang[degree]//8,500 step
	
	setVel( Ang_deg2step( conf.motor.h_angvel ), 23 );			// 速度=200 [deg/s]
	setStartVelRate( Ang_deg2step( conf.motor.h_angacc ), 27 );	// 起動・変速レート=200 [deg/s^2]
	setStopVelRate( Ang_deg2step( conf.motor.h_angacc ), 31 );	// 停止レート=200 [deg/s^2]

	setCurrentRate( 0x000003E8, 31 );// 運転電流=100.0 %
	setTrigger( 1, 35 );		// 反映トリガ=1:全データ反映
	setErrChk_send( 39 );		// エラーチェック
	
	send_buffer_length = 41;
	rev_buffer_length = 8;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	setChkCmd( );
	DEBUG_PRINT( printf( "[chk ] " ) );
	DEBUG_PRINT( printChkCommand( ) );
	
	usleep( 2 * 1000 );	//2ms
	bool ret = false;
	while( !ret ){
		usleep( 1 * 1000 );
		ret = chkResponse( );
	}
	usleep( 2 * 1000 );
	
	return true;
}
int ModbusRTU_DirectOrder::getStep( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x03, 1 );// ファンクションコード=0003h 保持レジスタの読み出し
	setRegAddr( 0x00CC, 2 );	// 保持レジスタ先頭アドレス=00CCh
	setRegAddrNum( 2, 4 );		// 保持レジスタから呼び出す数=2個
	setErrChk_send( 6 );			// エラーチェック

	send_buffer_length = 8;
//	rev_buffer_length = 9;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	usleep( 1 * 1000 );	//1ms
	bool ret = false;
	while( !ret ){
		usleep( 1 * 1000 );	//1ms
		ret = readCommand( );
//		printReadCommand( );
	}
	usleep( 1 * 1000 );	// 1ms
	
	int hi_2 = read_cmd[ 3 ];
	int hi_1 = read_cmd[ 4 ];
	int lo_2 = read_cmd[ 5 ];
	int lo_1 = read_cmd[ 6 ];
	int step = ( hi_2 << 24 ) + ( hi_1 << 16 ) + ( lo_2 << 8 ) + lo_1;

	return step;
}
double ModbusRTU_DirectOrder::getTorque( int id )
{
	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x03, 1 );// ファンクションコード=0003h 保持レジスタの読み出し
	setRegAddr( 0x00D6, 2 );	// 保持レジスタ先頭アドレス=00D6h
	setRegAddrNum( 2, 4 );		// 保持レジスタから呼び出す数=2個
	setErrChk_send( 6 );			// エラーチェック

	send_buffer_length = 8;
//	rev_buffer_length = 9;
	DEBUG_PRINT( printf( "[send] " ) );
	DEBUG_PRINT( printSendCommand( ) );
	sendCommand( );

	usleep( 1 * 1000 );	// 1ms
	bool ret = false;
	while( !ret ){
		ret = readCommand( );
//		printReadCommand( );
		usleep( 1 * 1000 );	//1ms
	}
	usleep( 1 * 1000 );	// 1ms
	
	int hi_2 = read_cmd[ 3 ];
	int hi_1 = read_cmd[ 4 ];
	int lo_2 = read_cmd[ 5 ];
	int lo_1 = read_cmd[ 6 ];
	int step = ( hi_2 << 24 ) + ( hi_1 << 16 ) + ( lo_2 << 8 ) + lo_1;
//	printf( "%d\n", step );

	return ( double )step;
}
double ModbusRTU_DirectOrder::getPosition( int id )
{
	int step = getStep( id );
	double pos = ( double )step / 1000.0;
	return pos;
}
static double calcHandleAng( double ang );
double ModbusRTU_DirectOrder::getAngle( void )
{
	int step = getStep( _HANDLE );
	double ang = calcHandleAng( ( ( double )step / 100.0 ) );
	return ang;
}
// *****************************************************************
// 間接参照方式用クラス
// *****************************************************************
int ModbusRTU_IndirectOrder::getStep( void )
{
	int hi_2 = read_cmd[ 3 ];
	int hi_1 = read_cmd[ 4 ];
	int lo_2 = read_cmd[ 5 ];
	int lo_1 = read_cmd[ 6 ];
	int step = ( hi_2 << 24 ) + ( hi_1 << 16 ) + ( lo_2 << 8 ) + lo_1;

	return step;
}
int ModbusRTU_IndirectOrder::getTorque( void )
{
	int hi_2 = read_cmd[ 7 ];
	int hi_1 = read_cmd[ 8 ];
	int lo_2 = read_cmd[ 9 ];
	int lo_1 = read_cmd[ 10 ];
	int tau = ( hi_2 << 24 ) + ( hi_1 << 16 ) + ( lo_2 << 8 ) + lo_1;

	return tau;
}
double ModbusRTU_IndirectOrder::getTorque( int id )
{
	double val;
	if( id == 1 ){
		val = state.accel.torque;
	} else if( id == 2 ){
		val = state.handle.torque;
	} else if( id == 3 ){
		val = state.lever.torque;
	} else {
		fprintf( stderr, "Error! ModbusRTU_TempRef::getTorque. %d\n", id );
		exit( EXIT_FAILURE );
	}
	return val;
}
double ModbusRTU_IndirectOrder::getPosition( int id )
{
	double val;
	if( id == 1 ){
		val = state.accel.pos;
	} else if( id == 3 ){
		val = state.lever.pos;
	} else {
		fprintf( stderr, "Error! ModbusRTU_TempRef::getPosition.%d\n", id );
		exit( EXIT_FAILURE );
	}
	return val;
}

int ModbusRTU_IndirectOrder::getAlarmCode( void )
{
	int hi_2 = read_cmd[ 11 ];
	int hi_1 = read_cmd[ 12 ];
	int lo_2 = read_cmd[ 13 ];
	int lo_1 = read_cmd[ 14 ];
	int tau = ( hi_2 << 24 ) + ( hi_1 << 16 ) + ( lo_2 << 8 ) + lo_1;
//printf("%02X%02X%02X%02X:%d\n",read_cmd[ 11 ],read_cmd[ 12 ],read_cmd[ 13 ],read_cmd[ 14 ],tau);
	return tau;
}
int ModbusRTU_IndirectOrder::getInfoCode( void )
{
	int hi_2 = read_cmd[ 15 ];
	int hi_1 = read_cmd[ 16 ];
	int lo_2 = read_cmd[ 17 ];
	int lo_1 = read_cmd[ 18 ];
	int step = ( hi_2 << 24 ) + ( hi_1 << 16 ) + ( lo_2 << 8 ) + lo_1;
//printf("%02X%02X%02X%02X:%d\n",read_cmd[ 15 ],read_cmd[ 16 ],read_cmd[ 17 ],read_cmd[ 18 ],step);
	return step;
}
int ModbusRTU_IndirectOrder::getAlarmCode( int id )		// アラーム
{
	int val;
	if( id == 1 ){
		val = state.accel.alarm;
	} else if( id == 2 ){
		val = state.handle.alarm;
	} else if( id == 3 ){
		val = state.lever.alarm;
	} else {
		fprintf( stderr, "Error! ModbusRTU_TempRef::getAlarmCode.\n" );
		exit( EXIT_FAILURE );
	}
	return val;
}
int ModbusRTU_IndirectOrder::getInfoCode( int id )		// インフォメーション
{
	int val;
	if( id == 1 ){
		val = state.accel.info;
	} else if( id == 2 ){
		val = state.handle.info;
	} else if( id == 3 ){
		val = state.lever.info;
	} else {
		fprintf( stderr, "Error! ModbusRTU_TempRef::getInfoCode.\n" );
		exit( EXIT_FAILURE );
	}
	return val;
}

bool ModbusRTU_IndirectOrder::sendPos( int id, double pos )
{
	int max = _ACCEL_POS_MAX;
	int min = _ACCEL_POS_MIN;
	if( id == _LEVER ){
		max = _LEVER_POS_MAX;
		min = _LEVER_POS_MIN;
	}
	if( pos >= max ) pos = max;
	if( pos <= min ) pos = min;
//	printf("pos=%f\n",pos);

	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x17, 1 );	// ファンクションコード=0017h
	setRegAddr( 0x1352, 2 );	// 読み込みレジスタ先頭アドレス=1352h
	setRegAddrNum( 8, 4 );		// 読み込みレジスタ数=8個
	setRegAddr( 0x1340, 6 );	// 書き込みレジスタ先頭アドレス=1340h
	setRegAddrNum( 18, 8 );		// 書き込みレジスタ数=18個
	setByteNum( 36, 10 );		// 書き込みバイト数=36 Byte
	setDriveData( 0, 11 );		// 運転データNo.=0
	setDriveMethod( 1, 15 );	// 方式=1:絶対位置決め
//	setDriveMethod( 2, 15 );	// 方式=2:相対位置決め(指令位置基準)
	setPos( Pos_mm2step( pos ), 19 );	// 位置=pos[mm]
	
	setVel( Pos_mm2step( conf.motor.a_vel ), 23 );			// 速度=600[mm/s]
	setStartVelRate( Pos_mm2step( conf.motor.a_acc ), 27 );	// 起動・変速レート=600[mm/s^2]
	setStopVelRate( Pos_mm2step( conf.motor.a_acc ), 31 );	// 停止レート=600[mm/s^2]

	setCurrentRate( 0x000003E8, 35 );// 運転電流=100.0 %
	setTrigger( 1, 39 );			// 反映トリガ=1:全データ反映
	setTransSite( 0, 43 );			// 運転転送先
	setErrChk_send( 47 );//	36+11	// エラーチェック

	send_buffer_length = 49;
	rev_buffer_length = 21;	// 現在位置・トルク・アラーム・インフォメーションの場合	

	DEBUG_PRINT( printf( "[send] " ) );
	sendCommand( );
	DEBUG_PRINT( printSendCommand( ) );

	usleep( 2 * 1000 );	//2ms
	bool ret = false;
	while( !ret ){
//		DEBUG_PRINT( printf( "[read] " ) );
		ret = readCommand( );
//		DEBUG_PRINT( printReadCommand( ) );
		usleep( 1 * 1000 );	//1ms
	}
	usleep( 2 * 1000 );	//2ms
	
	int step = getStep( );
	int torque = getTorque( );
	if( id == _ACCEL ){
		state.accel.input = pos;
		state.accel.pos = ( double )step / 1000.0;
		state.accel.torque = ( double )torque;
		state.accel.alarm = getAlarmCode( );
		state.accel.info = getInfoCode( );
	} else if( id == _LEVER ){
		state.lever.input = pos;
		state.lever.pos = ( double )step / 1000.0;
		state.lever.torque = ( double )torque;
		state.lever.alarm = getAlarmCode( );
		state.lever.info = getInfoCode( );
	}
	return true;
}

bool ModbusRTU_IndirectOrder::sendAng( int id, double ang )
{
	if( ang >= 180.0 ) ang = 179.0;
	if( ang <= -180.0 ) ang = -179.0;
	state.handle.input = ang;

	setDeviceID( id, 0 );		// 号機番号=1
	setFunctionCode( 0x17, 1 );	// ファンクションコード=0017h
	setRegAddr( 0x1352, 2 );	// 読み込みレジスタ先頭アドレス=1352h
	setRegAddrNum( 8, 4 );		// 読み込みレジスタ数=8個
	setRegAddr( 0x1340, 6 );	// 書き込みレジスタ先頭アドレス=1340h
	setRegAddrNum( 18, 8 );		// 書き込みレジスタ数=18個
	setByteNum( 36, 10 );		// 書き込みバイト数=36 Byte
	setDriveData( 0, 11 );		// 運転データNo.=0
//	setDriveMethod( 1, 15 );	// 方式=1:絶対位置決め
//	setDriveMethod( 2, 15 );	// 方式=2:相対位置決め(指令位置基準)
	setDriveMethod( 3, 15 );	// 方式=2:相対位置決め(検出位置基準)
//	setDriveMethod( 22, 15 );	// 方式=2:相対位置決め押し当て(検出位置基準)
	setPos( Ang_deg2step( ang ), 19 );			// 位置=pos[mm]//8,500 step
	
	setVel( Ang_deg2step( conf.motor.h_angvel ), 23 );			// 速度=800 [deg/s]
	setStartVelRate( Ang_deg2step( conf.motor.h_angacc ), 27 );	// 起動・変速レート=800 [deg/s^2]
	setStopVelRate( Ang_deg2step( conf.motor.h_angacc ), 31 );	// 停止レート=800 [deg/s^2]

	setCurrentRate( 0x000003E8, 35 );// 運転電流=100.0 %
	setTrigger( 1, 39 );			// 反映トリガ=1:全データ反映
	setTransSite( 0, 43 );			// 運転転送先
	setErrChk_send( 47 );			// エラーチェック
	
	send_buffer_length = 49;
	rev_buffer_length = 21;	// 現在位置・トルク・アラーム・インフォメーションの場合

	DEBUG_PRINT( printf( "[send] " ) );
	sendCommand( );
	DEBUG_PRINT( printSendCommand( ) );
	
	usleep( 2 * 1000 );	//1ms
	bool ret = false;
	while( !ret ){
		ret = readCommand( );
//		printReadCommand( );
		usleep( 1 * 1000 );	//1ms
	}
	usleep( 2 * 1000 );	//1ms
	
	int step = getStep( );
	state.handle.ang = calcHandleAng( ( ( double )step / 100.0 ) );
	state.handle.torque = ( double )getTorque( );
	state.handle.alarm = getAlarmCode( );
	state.handle.info = getInfoCode( );	
	
	return true;
}
// 絶対角度で計算する
static int handle_area = 0;			// ( -2, -1, 0, 1, 2 )
static double trans_theta = 90.0;
static double calcHandleAng( double ang )
{
	double outp;
	outp = ang;
	if( handle_area == 0 ){
		if( trans_theta <= outp ) handle_area = 1;
		if( -1.0*trans_theta >= outp ) handle_area = -1;
		return ang;
	} else {
		if( handle_area > 0 ){
			double Thre0 = ( double )( handle_area - 1 ) * 180.0 + trans_theta;
			double Thre1 = ( double )handle_area * 180.0 + trans_theta;
			if( ang > 0 ){

				if( ( handle_area % 2 ) != 0 ){
					double cnt = ( double )( handle_area - 1 ) / 2;
					outp = cnt * 360.0 + ang;
				} else {
					outp = ( ( double )handle_area / 2 ) * 360.0 + ang;
				}

			} else if( ang < 0 ){

				if( ( handle_area % 2 ) != 0 ){
					double cnt = 1 + ( ( double )( handle_area - 1 ) / 2 );
					outp = cnt * 360.0 + ang;

				} else {

					outp = ( ( double )handle_area / 2 ) * 360.0 + ang;

				}
			}
			if( Thre0 > outp ) handle_area--;
			if( Thre1 < outp ) handle_area++;

		} else if( handle_area < 0 ){

			double Thre0 = ( double )( handle_area + 1 ) * 180.0 - trans_theta;
			double Thre1 = ( double )handle_area * 180.0 - trans_theta;
			
			if( ang <= 0 ){
				if( ( handle_area % 2 ) != 0 ){
					double cnt = ( double )( handle_area + 1 ) / 2;
					outp = cnt * 360.0 + ang;
				} else {
					outp = ( ( double )handle_area / 2 ) * 360.0 + ang;
				}
			} else if( ang > 0 ){
				
				if( ( handle_area % 2 ) != 0 ){
					double cnt = -1 + ( ( double )( handle_area + 1 ) / 2 );
					outp = cnt * 360.0 + ang;

				} else {

					outp = ( ( double )handle_area / 2 ) * 360.0 + ang;
				}
			}
			if( Thre0 > outp ) handle_area--;
			if( Thre1 < outp ) handle_area++;
		}
	}

	return outp;
}
void ModbusRTU_IndirectOrder::resetAngle( void )
{
	handle_area = 0;
}
