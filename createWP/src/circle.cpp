#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*-PI < theta < PIに調整する*/
static double trans_q( double theta )
{
    while( theta > M_PI )
        theta -= 2.0 * M_PI;
    while( theta < -M_PI )
        theta += 2.0 * M_PI;
    return theta;
}

int main( void )
{
	double cPos[ 3 ];	// 現在位置・姿勢
	cPos[ 0 ] = -46459.3825; 	// x
	cPos[ 1 ] = -14689.9411;	// y
	cPos[ 2 ] = 1.1382;	// theta
	
	double radius = 4;//3;//5;	// 半径[m]
	
	double center[ 2 ];
	center[ 0 ] = cPos[ 0 ] - radius * sin( cPos[ 2 ] );
	center[ 1 ] = cPos[ 1 ] + radius * cos( cPos[ 2 ] );
//	printf(	"%f %f\n", center[ 0 ], center[ 1 ] );

	double Pos[ 2 ];	// 円の軌跡
	int i = 0;
	double sth = -1.0 * trans_q( ( M_PI/2 ) - cPos[ 2 ] );
	double eth = eth + ( 2.0 * M_PI );
//	printf( "start %f %f\n", ( 0.3 / radius ), ( 2.0 * M_PI ) );	
//	for( double th = 0 ; th < ( M_PI/4 ) ; th += ( 0.5 / radius ) ){
	for( double th = sth ; th < eth ; th += ( 0.5 / radius ) ){
		Pos[ 0 ] = center[ 0 ] + radius * sin( th );
		Pos[ 1 ] = center[ 1 ] - radius * cos( th );
		printf( "%d %15.4f %15.4f\n", i, Pos[ 0 ], Pos[ 1 ] );
		i++;
	}
	
	return 0;
}
