#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <stdio.h>
#include <math.h>
#include <ssmtype/spur-odometry.h>

using namespace std;

#define STRLEN	256

#define _ROLL		0
#define _PITCH		1
#define _YAW		2
#define _X			0
#define _Y			1
#define _Z			2
#define _V			2

#define DEG2RAD( x ) ( ( x ) * M_PI / 180)  // 度からラジアン
#define RAD2DEG( x ) ( ( x ) * 180 / M_PI)  // ラジアンから度
#define SIGN( x )	( ( x < 0 ) ? -1 : 1 )

double trans_q( double theta );

FILE* isValidFile( const char* filename, const char* mode, const char* ext );
char* getWord( FILE *fp );

void Gprint( const char *str );
void Rprint( const char *str );

bool kbhit( void );

#endif
