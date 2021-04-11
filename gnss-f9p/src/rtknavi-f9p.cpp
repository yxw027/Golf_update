//-------------------------------------------------
// rtknavi-f9p.cpp
// 
// T.Hasegawa
// Since: 2017-10-29
// Update: 2019/07/09
// Update: 2019/08/17
// Modify: 2020/12/10
// Update: 2021/03/16
//-------------------------------------------------

#include <stdio.h>
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
#include <time.h>

#include "gnss-f9p.hpp"
#include "rtknavi-f9p.hpp"
#include "utility.hpp"
#include "framework.hpp"

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT( x ) ( x )
#else
#define DEBUG_PRINT( x )
#endif

void RTKNAVI_F9P_BASE::openGNSSReceiver( const char *dev )
{
	sprintf( device, "%s", dev );
	printf( "Device Name = %s\n", device );

//	if( !( fd = open( device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL ) ) ){		 // デバイスをオープンする
	if( !( fd = open( device, O_RDWR | O_NOCTTY) ) ){		 // デバイスをオープンする
		fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]:fail to open F9P GNSS Receiver\n" );
		exit( EXIT_FAILURE );
	}
	ioctl( fd, TCGETS, &oldtio );		// 現在のシリアルポートの設定を待避

	bzero( &newtio, sizeof( newtio ) );
	newtio = oldtio;					// ポートの設定をコピー

	newtio.c_cflag = CREAD;               // 受信有効
	newtio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
	newtio.c_cflag += CS8;                 // データビット:8bit
	newtio.c_cflag += 0;                   // ストップビット:1bit
	newtio.c_cflag += 0;                   // パリティ:None

	cfmakeraw( &newtio );
	cfsetispeed( &newtio, BAUDRATE );
	cfsetospeed( &newtio, BAUDRATE );
	ioctl( fd, TCSETS, &newtio );		// ポートの設定を有効にする
}
void RTKNAVI_F9P_BASE::closeGNSSReceiver( void )
{
	ioctl( fd, TCSETS, &oldtio );	// ポートの設定を元に戻す
	close( fd );					// デバイスのクローズ
}
#define STR_MAX 256
#define SCI_VMIN 100000 //[us]
#define SCI_TIMEOUT 5000*SCI_VMIN //[us]
#define rev_buffer_length	256
static char str_gga[ rev_buffer_length ] = { '0' };
int RTKNAVI_F9P_BASE::readCommand_gga( void )
{
	int cnt_gga = 0;
	int ret;
	int cnt_timeout = 0;
	bool update = true;

	while( update ){
		unsigned char tmp[ rev_buffer_length ];
		// GNSSデータの読み込み
		ret = read( fd, &tmp, rev_buffer_length-1 );
//		fprintf( stderr, "size=%d, %s\n",ret, tmp );
		
		if( ret < 0 ){
			fprintf( stderr, "[readCommand]ret1=%d\n",ret );
			return -1;
		
		} else if( ret == 0 ){
			cnt_timeout++;
		
		} else {
//			fprintf( stderr, "**size=%d, %s\n",ret, tmp );
			char key_word[ 2 ]; // "$"の検出用変数
			for( int i = 0 ; i < ret ; i++ ){
				key_word[ 0 ] = tmp[ i ];
//				fprintf( stderr, "key_word=%c ",key_word[0] );
				// "$"の検出
				if( !strncmp( key_word, "$", 1 ) ){
//					fprintf( stderr, "key_word=%c ",key_word[0] );
					cnt_gga = 0;
					int j;
					for( j = i ; j < i+6 ; j++ ){
						str_gga[ cnt_gga ] = tmp[ j ];
						cnt_gga++;
					}
//					if( !strncmp( str_gga, "$GNRMC", 6 ) ){

					if( !strncmp( str_gga, "$GNGGA", 6 ) ){
//						fprintf( stderr, "str_gga=%s\n",str_gga );
						for( int t = j ; t < ret ; t++ ){
							str_gga[ cnt_gga ] = tmp[ t ];
							if( tmp[ t ] == 0x0a ){
//								fprintf( stderr, "**[%d]size=%d, %s\n",i,ret, tmp );
								str_gga[ cnt_gga+1 ] = '\0';
								update = false;
								break;
							}
							cnt_gga++;
						}
//						fprintf( stderr, "size=%d, %s\n",ret, str_gga );
						i = ret;
					}
				}
			}
		}
		if( cnt_gga >= STR_MAX ){
			fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Receive too many data.\n" );
			return -1;
		}

//		if( cnt_timeout * SCI_VMIN > SCI_TIMEOUT ){
		if( cnt_timeout > SCI_TIMEOUT ){
			printf( "[%d/%d] ",cnt_gga, rev_buffer_length );
//			printReadCommand( );
			fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Time Out.\n" );
			return -1;
		}
	}

	return cnt_gga;
}
/*
static char str_rmc[ rev_buffer_length ] = { '0' };
int RTKNAVI_F9P_BASE::readCommand_rmc( void )	// 日付取得用
{
	int cnt_rmc = 0;
	int ret;
	int cnt_timeout = 0;
	bool update = true;

	while( update ){
		unsigned char tmp[ rev_buffer_length ];
		// GNSSデータの読み込み
		ret = read( fd, &tmp, rev_buffer_length-1 );
//		fprintf( stderr, "size=%d, %s\n",ret, tmp );
		
		if( ret < 0 ){
			fprintf( stderr, "[readCommand]ret1=%d\n",ret );
			return -1;
		
		} else if( ret == 0 ){
			cnt_timeout++;
		
		} else {
//			fprintf( stderr, "**size=%d, %s\n",ret, tmp );
			char key_word[ 2 ]; // "$"の検出用変数
			for( int i = 0 ; i < ret ; i++ ){
				key_word[ 0 ] = tmp[ i ];
//				fprintf( stderr, "key_word=%c ",key_word[0] );
				// "$"の検出
				if( !strncmp( key_word, "$", 1 ) ){
//					fprintf( stderr, "key_word=%c ",key_word[0] );
					cnt_rmc = 0;
					int j;
					for( j = i ; j < i+6 ; j++ ){
						str_rmc[ cnt_rmc ] = tmp[ j ];
						cnt_rmc++;
					}
					if( !strncmp( str_rmc, "$GNRMC", 6 ) ){
//						fprintf( stderr, "str_rmc=%s\n",str_rmc );
						for( int t = j ; t < ret ; t++ ){
							str_rmc[ cnt_rmc ] = tmp[ t ];
							if( tmp[ t ] == 0x0a ){
//								fprintf( stderr, "**[%d]size=%d, %s\n",i,ret, tmp );
								str_rmc[ cnt_rmc+1 ] = '\0';
								update = false;
								break;
							}
							cnt_rmc++;
						}
//						fprintf( stderr, "size=%d, %s\n",ret, str_rmc );
						i = ret;
					}
				}
			}
		}
		if( cnt_rmc >= STR_MAX ){
			fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Receive too many data.\n" );
			return -1;
		}

//		if( cnt_timeout * SCI_VMIN > SCI_TIMEOUT ){
		if( cnt_timeout > SCI_TIMEOUT ){
			printf( "[%d/%d] ",cnt_rmc, rev_buffer_length );
//			printReadCommand( );
			fprintf( stderr, "[\033[1m\033[31mERROR\033[30m\033[0m]: Time Out.\n" );
			return -1;
		}
	}

	return cnt_rmc;
}
*/
int RTKNAVI_F9P_BASE::readCommand( void )
{
	// GPSから受信
//	int size_rmc = readCommand_rmc( );
//	printf( "rmc[%d]%s", size_rmc, str_rmc );
	int size_gga = readCommand_gga( );
//	printf( "gga[%d]%s", size_gga, str_gga );
	return size_gga;
}
void RTKNAVI_F9P::printRTKGNSS( rtk_gnss_f9p *data )
{
	if( data->status ){
		fprintf( stderr, "%f ", data->utc_time );	//1 
		fprintf( stderr, "%15.8f ", data->latitude );	//2 (deg)
		fprintf( stderr, "%15.8f ", data->longitude );	//3 (deg)
		fprintf( stderr, "%15.8f ", data->height );		//4 (m)
		fprintf( stderr, "%d ", data->posStatus );	// 5 FIX or FLOAT
		fprintf( stderr, "%f ", data->ecef.x );	// 6
		fprintf( stderr, "%f ", data->ecef.y );	// 7
		fprintf( stderr, "%f ", data->ecef.z );	// 8
		fprintf( stderr, "%f ", data->enu.x );	// 9
		fprintf( stderr, "%f ", data->enu.y );	// 10
		fprintf( stderr, "%f ", data->enu.z );	// 11
		
		fprintf( stderr, "%f ", data->elevation );		//12 (m)
		fprintf( stderr, "%f ", data->Geoid_height );	//13 (m)
		fprintf( stderr, "%d ", trans.getCoodinatorID( ) );	// 14

		fprintf( stderr, "\n" );
	}
}

void TransCoodinator::setCoodinatorID( int c )
{
	coodinator_id = c;
	double phi, ramda, height;
	switch( c ){
	case 1:
		phi    = 33.0000;
		ramda  = 129.5000;
		height = 0;
		break;
	case 2:
		phi    = 33.0000;
		ramda  = 131.0000;
		height = 0;
		break;
	case 3:
		phi    = 36.0000;
		ramda  = 132.1666;
		height = 0;
		break;
	case 4:
		phi    = 33.0000;
		ramda  = 133.5000;
		height = 0;
		break;
	case 6:
		phi    = 36.0000;
		ramda  = 136.0000;
		height = 0;
		break;
	case 7:
		phi    = 36.0000;
		ramda  = 137.1666;
		height = 0;
		break;
	case 8:
		phi    = 36.0000;
		ramda  = 138.5000;
		height = 0;
		break;
	case 9:
		phi    = 36.0000;
		ramda  = 139.8333;
		height = 0;
		break;
	default:
		printf( "Error. Cooddinate ID...\n" );
		exit( EXIT_FAILURE );
	}
	
	ecef_0 = blh2ecef( phi, ramda, height );
}
bool RTKNAVI_F9P::decodeRTKGNSS( rtk_gnss_f9p *dst )
{
//	decodeRTKGNSS_rmc( dst );
	bool ret = decodeRTKGNSS_gga( dst );
	return ret;
}
#define BUF_SIZE	1024
bool RTKNAVI_F9P::decodeRTKGNSS_gga( rtk_gnss_f9p *dst )
{
	bool status = true;
	dst->status = status;
	// GPSから受信
	int size = readCommand( );
//	printf( "[%d]%s", size, str_gga );
	if( size >= rev_buffer_length || size < 0 ){	// 受信データ
		if( size == -1 ){
			fprintf( stderr, "Read Failed : %s\n", device );
			exit( EXIT_FAILURE );
		}
		dst->status = false;
		return false;
		
	} else {
		char *tmp_gga;
//########## 第1項目 #########
	//***** $GNGGA *****
		tmp_gga = strtok( str_gga, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
//########## 第2項目 #########		
	//***** UTC time (GNGGA)*****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
		// 秒に変換
		double hour = ( double )( ( tmp_gga[ 0 ] - '0' ) * 10 + ( tmp_gga[ 1 ] - '0' ) );
		hour *= 3600.0;
		double min = ( double )( ( tmp_gga[ 2 ] - '0' ) * 10 + ( tmp_gga[ 3 ] - '0' ) );
		min *= 60.0;
		double sec = ( double )( ( tmp_gga[ 4 ] - '0' ) * 10 + ( tmp_gga[ 5 ] - '0' ) );
		double decimal = ( double )( ( tmp_gga[ 7 ] - '0' ) * 10 + ( tmp_gga[ 8 ] - '0' ) ) * 0.01;
		dst->utc_time = hour + min + sec + decimal;
//		fprintf( stderr, "utc_time=%f\n", dst->utc_time );
//########## 第3項目 #########	
	//***** latitude (GNGGA)*****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//			puts( tmp_gga );
			int len = strcspn( tmp_gga, "." );
			int max_len = strlen( tmp_gga );
//			printf("%c, %d %d\n",tmp[0], len, max_len );
			char dd[ 10 ] = {'0'}, mm[ 30 ] = {'0'};
			for( int i = 0 ; i < len-2 ; i++ ){
				dd[ i ] = tmp_gga[ i ];
			}
			double ddreal = atof( dd );
//			printf("%s : %f\n",dd, ddreal);
			for( int i = len-2, j = 0 ; i < max_len ; i++, j++ ){
				mm[ j ] = tmp_gga[ i ];
			}
			double mmreal = atof( mm );
//			printf("%s : %lf : %lf\n",mm, mmreal, mmreal/60);
			dst->latitude = ddreal + ( mmreal / 60.0 );
//			printf( "%f\n", dst->latitude );
	// ***** N *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
	//***** longitude *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//			puts( tmp_gga );
			len = strcspn( tmp_gga, "." );
			max_len = strlen( tmp_gga );
//			printf("%c, %d %d\n",tmp[0], len, max_len );
			for( int i = 0 ; i < len-2 ; i++ ){
				dd[ i ] = tmp_gga[ i ];
			}
			ddreal = atof( dd );
//			printf("%s : %f\n",dd, ddreal);
			for( int i = len-2, j = 0 ; i < max_len ; i++, j++ ){
				mm[ j ] = tmp_gga[ i ];
			}
			mmreal = atof( mm );
//			printf("%s : %lf\n",mm, mmreal);
			dst->longitude = ddreal + ( mmreal / 60.0 );
//			printf( "%f\n", dst->longitude );
	// ***** E *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
	//***** Q (Fix or Float) ***** 測位のステータス
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
		dst->posStatus = atoi( tmp_gga );//	puts( tmp_gga );
	// ***** 使用衛星数 *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
	// ***** HDOP 衛星配置による精度低下率 *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
	// ***** アンテナの平均水面からの高さ（海抜高さ） *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
		dst->elevation = atof( tmp_gga );//		puts( tmp_gga );
	// ***** M *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
	//***** height ***** ジオイド高（楕円体から平均水面までの高さ）
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
		dst->Geoid_height = atof( tmp_gga );//		puts( tmp_gga );
#ifdef USE_ELEVATION
		dst->height = dst->elevation; // 標高
#else
		dst->height = dst->elevation + dst->Geoid_height; // 楕円対高 = 標高 + ジオイド高
#endif

	// ***** M *****
		tmp_gga = strtok( NULL, "," );
		if( tmp_gga == NULL ){
			status = false;
			dst->status = status;
			return status;
		}
//		puts( tmp_gga );
		
	// ECEF (x, y, z)
		my_vector_t ecef;
		ecef = trans.blh2ecef( dst->latitude, dst->longitude, dst->height );
		dst->ecef.x = ecef.a[ _X ];
		dst->ecef.y = ecef.a[ _Y ];
		dst->ecef.z = ecef.a[ _Z ];
	// ENU (x, y, z)
		my_vector_t enu = trans.ecef2enu( ecef, trans.getECEF0( ) );
		dst->enu.x = enu.a[ _X ];
		dst->enu.y = enu.a[ _Y ];
#ifdef USE_ELEVATION
		dst->enu.z = dst->elevation; // 標高
#else
		dst->enu.z = enu.a[ _Z ];
#endif
	}

	return status;
}
/*
bool RTKNAVI_F9P::decodeRTKGNSS_rmc( rtk_gnss_f9p *dst )
{
	bool status = true;
	// GPSから受信
	int size = readCommand( );
//	printf( "[%d]%s", size, str_rmc );
	if( size >= rev_buffer_length || size < 0 ){	// 受信データ
		if( size == -1 ){
			fprintf( stderr, "Read Failed : %s\n", device );
			exit( EXIT_FAILURE );
		}
		return false;
		
	} else {
		char *tmp_rmc;
//########## 第1項目 #########
	//***** $GNRMC *****
		tmp_rmc = strtok( str_rmc, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第2項目 #########		
	//***** UTC time (GNRMC) *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		sprintf( dst->utc_time, "%s", tmp_rmc );
//		puts( dst->utc_time );
//		puts( tmp_rmc );
//########## 第3項目 #########		
	//***** ステータス (GNRMC) *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第4項目 #########	
	//***** latitude (GNRMC) *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第5項目 #########	
	// ***** N *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第6項目 #########	
	//***** longitude *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第7項目 #########	
	// ***** E *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第8項目目 #########	
	//***** 移動の速度 ***** 
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		puts( tmp_rmc );
//########## 第9項目目 #########	
	// ***** 移動の真方位 *****
		//tmp_rmc = strtok( NULL, "," );
		//if( tmp_rmc == NULL ){
			//status = false;
			//return status;
		//}
		//puts( tmp_rmc );
//########## 第10項目目 #########	
	// ***** UTC日付 *****
		tmp_rmc = strtok( NULL, "," );
		if( tmp_rmc == NULL ){
			status = false;
			return status;
		}
//		sprintf( dst->date, "%s", tmp_rmc );
//		puts( dst->date );
//		puts( tmp_rmc );
	}

	return status;
}
*/
//--- ECEF座標を水平線座標(ENU)へ変換する 
static matrix matmat( matrix *m1, matrix *m2 );
static my_vector_t matvec( matrix inmat, my_vector_t v1 );
static matrix rotx( double sita );
static matrix roty( double sita );
static matrix rotz( double sita );
my_vector_t TransCoodinator::ecef2enu( my_vector_t dest, my_vector_t origin )
{
	my_vector_t mov, ret, blh;
	matrix rotyp, rotzp1, rotzp2;
	matrix mat_conv1, mat_conv2;

	origin.n = 3;
	origin.err = false;
	mov.n = 3;
	mov.err = false;
	ret.n = 3;
	ret.err = false;

	blh = ecef2blh( origin );

	rotzp1 = rotz( 90.0 );
	rotyp = roty( 90.0 - blh.a[ 0 ] );
	rotzp2 = rotz( blh.a[ 1 ] );

	mat_conv1 = matmat( &rotzp1, &rotyp );
	mat_conv2 = matmat( &mat_conv1, &rotzp2 );

	for( int i = 0 ; i < 3 ; i++ )
		mov.a[ i ] = dest.a[ i ] - origin.a[ i ];

	ret = matvec( mat_conv2, mov );

	return ret;
}
// ECEF座標からWGS84の{緯度,経度,楕円体高}へ変換 
//--- WGS84座標パラメータ 
#define SQR( x )	( ( x ) * ( x ) )			//--- 2乗
#define CUB( y )	( ( y ) * ( y ) * ( y ) )	//--- 3乗
#define A 			6378137.0		// Semi-major axis 
#define ONE_F		298.257223563	// 1/F 
#define B			( A * ( 1.0 - 1.0 / ONE_F ) )
#define E2			( ( 1.0 / ONE_F ) * ( 2 - ( 1.0 / ONE_F ) ) )
#define ED2			( E2 * A * A / ( B * B ) )
#define NN( p )		( A / sqrt( 1.0 - ( E2 ) * SQR( sin( p * M_PI / 180.0 ) ) ) )

my_vector_t TransCoodinator::ecef2blh( my_vector_t ec )
{
	my_vector_t blh;
	double phi, ramda, height, p;
	double x, y, z;
	double sita;

	ec.n = 3;
	blh.n = 3;
	x = ec.a[ 0 ], y = ec.a[ 1 ], z = ec.a[ 2 ];
	p = sqrt( x*x + y*y );
	sita = ( 180 / M_PI ) * atan2( ( z * A ), ( p * B ) );
	//--- 緯度 
	phi = ( 180 / M_PI ) * atan2( z + ED2 * B * ( CUB ( sin ( sita * M_PI / 180 ) ) ) , ( p - E2 * A * ( CUB ( cos ( sita * M_PI / 180 ) ) ) ) );

	//--- 経度 
	ramda = ( 180 / M_PI ) * atan2( y, x );

	//--- 高さ 
	height = ( p / cos( phi * M_PI / 180 ) ) - NN( phi );

	blh.a[ 0 ] = phi;
	blh.a[ 1 ] = ramda;
	blh.a[ 2 ] = height;

	return blh;
}
// 緯度,経度,高さからECEF座標に変換 
my_vector_t TransCoodinator::blh2ecef( double phi, double ramda, double height )
{
	my_vector_t ecef;

	ecef.n = 3;
	ecef.a[ 0 ] = ( NN( phi ) + height ) * cos( phi * M_PI / 180 ) * cos( ramda * M_PI / 180 );
	ecef.a[ 1 ] = ( NN( phi ) + height ) * cos( phi * M_PI / 180 ) * sin( ramda * M_PI / 180 );
	ecef.a[ 2 ] = ( NN( phi ) * ( 1.0 - E2 ) + height ) * sin( phi * M_PI / 180 );

	return ecef;
}
//--- 行列の積 
static matrix matmat( matrix *m1, matrix *m2 )
{
	int mcount;
	matrix ret_mat;

	ret_mat.err = false;
	if( ( m1->n > MAXN ) || ( m1->n < 0 ) || ( m1->m > MAXN ) || ( m1->m < 0 ) ) ret_mat.err = true;
	if( ( m2->n > MAXN ) || ( m2->n < 0 ) || ( m2->m > MAXN ) || ( m2->m < 0 ) ) ret_mat.err = true;

	if( ( m1->m ) != ( m2->n ) ) ret_mat.err = true;
	mcount = m1->m;
	if( ret_mat.err == true ){
		strcpy( ret_mat.message, "Something went wrong." );
		printf( "Something went wrong!\n" );
		ret_mat.n = 0;
		ret_mat.m = 0;
		return ret_mat;
	}

	ret_mat.n = m1->n;
	ret_mat.m = m2->m;

	for( int i = 0 ; i < ret_mat.n ; i++ ){
		for( int j = 0 ; j < ret_mat.m ; j++ )
			ret_mat.a[ i ][ j ] = 0.0;
	}

	for( int i = 0 ; i < ret_mat.n ; i++ ){
		for( int j = 0 ; j < ret_mat.m ; j++ ){
			for( int k = 0 ; k < mcount ; k++ ){
				ret_mat.a[ i ][ j ] += m1->a[ i ][ k ] * m2->a[ k ][ j ];
			}
		}
	}
	return ret_mat;
}
//--- 行列とベクトルの積 
static my_vector_t matvec( matrix inmat, my_vector_t v1 )
{
	my_vector_t ret_v;

	for( int i = 0 ; i < MAXN ; i++ )
		ret_v.a[ i ] = 0.0;

	ret_v.err = false;
	if( ( inmat.n > MAXN ) || ( inmat.n < 0 ) || ( inmat.m > MAXN ) || ( inmat.m < 0 ) ) ret_v.err = true;
	if( ( v1.n > MAXN ) || ( v1.n < 0 ) ) ret_v.err = true;
	if( ( v1.n ) != ( inmat.m ) ) ret_v.err = true;
	if( ret_v.err == true ){
		ret_v.n = false;
		return ret_v;
	}
	ret_v.n = inmat.n;
	for( int i = 0 ; i < inmat.n ; i++ ){
		for(int j = 0 ; j < inmat.m ; j++ )
			ret_v.a[ i ] = ret_v.a[ i ] + inmat.a[ i ][ j ] * v1.a[ j ];
	}
	return ret_v;
}
// x軸回りのsita度の回転変換：右ねじの方向 
static matrix rotx( double sita )
{
	matrix rota;

	rota.n = rota.m = 3;
	rota.a[ 0 ][ 0 ] = 1;
	rota.a[ 0 ][ 1 ] = 0;
	rota.a[ 0 ][ 2 ] = 0;
	rota.a[ 1 ][ 0 ] = 0;
	rota.a[ 1 ][ 1 ] = cos( sita * M_PI / 180.0 );
	rota.a[ 1 ][ 2 ] = sin( sita * M_PI / 180.0 );
	rota.a[ 2 ][ 0 ] = 0;
	rota.a[ 2 ][ 1 ] = -sin( sita * M_PI / 180.0 );
	rota.a[ 2 ][ 2 ] = cos( sita * M_PI / 180.0 );

	return rota;
}
static matrix roty( double sita )
{
	matrix rota;

	rota.n = rota.m = 3;

	rota.a[ 0 ][ 0 ] = cos( sita * M_PI / 180.0 );
	rota.a[ 0 ][ 1 ] = 0;
	rota.a[ 0 ][ 2 ] = -sin( sita * M_PI / 180.0 );
	rota.a[ 1 ][ 0 ] = 0;
	rota.a[ 1 ][ 1 ] = 1;
	rota.a[ 1 ][ 2 ] = 0;
	rota.a[ 2 ][ 0 ] = sin( sita * M_PI / 180.0 );
	rota.a[ 2 ][ 1 ] = 0;
	rota.a[ 2 ][ 2 ] = cos( sita * M_PI / 180.0 );

	return rota;
}
static matrix rotz( double sita )
{
	matrix rota;

	rota.n = rota.m = 3;

	rota.a[ 0 ][ 0 ] = cos( sita * M_PI / 180.0 );
	rota.a[ 0 ][ 1 ] = sin( sita * M_PI / 180.0 );
	rota.a[ 0 ][ 2 ] = 0;
	rota.a[ 1 ][ 0 ] = -sin( sita * M_PI / 180.0 );
	rota.a[ 1 ][ 1 ] = cos( sita * M_PI / 180.0 );
	rota.a[ 1 ][ 2 ] = 0;
	rota.a[ 2 ][ 0 ] = 0;
	rota.a[ 2 ][ 1 ] = 0;
	rota.a[ 2 ][ 2 ] = 1;

	return rota;
}
