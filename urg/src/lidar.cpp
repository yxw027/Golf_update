#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdexcept>
#include <getopt.h>
#include <math.h>
#include <ssm.hpp>
#include "urg.hpp"
#include "lidar.hpp"

bool lidar_Hokuyo::initialize( char *dev, bool flag )
{
	flag_intensity = flag;
	
	// device valiant
	S2Port *port;		// URG port

	// set device
	bool flag_ethernet = false;
	char* pt;
	// Open the port 
	if( ( pt = strchr( dev, ':' ) ) != NULL ){
		char *address, *port_number;
		address = strtok( dev, ":" );
		port_number = strtok( NULL, ":" );
		port = Scip2_OpenEthernet( address, atoi(port_number) );
		if( port != NULL ) flag_ethernet = true;
	}
	if( !flag_ethernet ){
		port = Scip2_Open( dev, B0 );
	}
	if( port == 0 ){
		fprintf( stderr, "ERROR: Failed to open device (%s).\n", dev );
		return false;
	}

	// get URG parameter 
	Scip2CMD_PP( port, &param );
	Scip2CMD_VV( port, &ver );	
	
	// Initialize buffer before getting scanned data
	S2Sdd_Init( &buf );
	// MS command 
	if( flag_intensity ){
		Scip2CMD_StartMS( port, param.step_min, param.step_max, 1, 0, 0, &buf, SCIP2_ENC_3X2BYTE );
	} else {
		Scip2CMD_StartMS( port, param.step_min, param.step_max, 1, 0, 0, &buf, SCIP2_ENC_3BYTE );
	}
	
	return true;
}
bool lidar_Hokuyo::getPointCloudData( urg_fs *urg )
{
	double length, angle;
	unsigned int point_num = 0;

	int ret = S2Sdd_Begin( &buf, &data );
	if( ret > 0 ){
		// ---- analyze data ---- 
		if( flag_intensity ){
			for( int i = 0 ; i < data->size ; i += 2 ){
				if( data->data[ i ] > param.dist_max || data->data[ i ] < param.dist_min ) continue;
				angle = ( double )( i/2 - param.step_front + param.step_min ) * 2.0 * M_PI / param.step_resolution;
				length = ( double )data->data[ i ] / 1000.0;
				urg->length[ point_num ] = length;
				urg->angle[ point_num ] = angle;
				urg->intensity[ point_num ] = data->data[ i + 1 ];
				point_num++;
			}
		} else {
			for( int i = 0 ; i < data->size ; i++ ){
				if( data->data[ i ] > param.dist_max || data->data[ i ] < param.dist_min ) continue;
				angle = ( double )( i - param.step_front + param.step_min ) * 2.0 * M_PI / param.step_resolution;
				length = ( double )data->data[ i ] / 1000.0;
				urg->length[ point_num ] = length;
				urg->angle[ point_num ] = angle;
				urg->intensity[ point_num ] = 0;
				point_num++;
			}
		}
		urg->size = point_num - 1;

		S2Sdd_End( &buf );
				
	} else if( ret == -1 ){
		fprintf( stderr, "ERROR: Fatal error occurred.\n" );
		return false;
	} else {
		return false;
	}
	return true;
}
void lidar_Hokuyo::getProperty( urg_property *urg )
{
	strncpy( urg->model, param.model, SCIP2_MAX_LENGTH );
	urg->dist_min = param.dist_min;
	urg->dist_max = param.dist_max;
	urg->step_resolution = param.step_resolution;
	urg->step_min = param.step_min;
	urg->step_max = param.step_max;
	urg->step_front = param.step_front;
	urg->revolution = param.revolution;

	printf( "\n<URG INFO>\n" );
	strncpy( urg->vender, ver.vender, SCIP2_MAX_LENGTH );
	strncpy( urg->product, ver.product, SCIP2_MAX_LENGTH );
	strncpy( urg->firmware, ver.firmware, SCIP2_MAX_LENGTH );
	strncpy( urg->protocol, ver.protocol, SCIP2_MAX_LENGTH );
	strncpy( urg->serialno, ver.serialno, SCIP2_MAX_LENGTH );
	printf( "vernder  : %s\n", urg->vender );
	printf( "product  : %s\n", urg->product );
	printf( "firmware : %s\n", urg->firmware );
	printf( "protocol : %s\n", urg->protocol );
	printf( "serial no: %s\n\n", urg->serialno );

	printf( "<%s>\n", urg->model );
	printf( "%5d # MINIMUM DISTANCE [mm]\n", urg->dist_min );
	printf( "%5d # MAXIMUM DISTANCE [mm]\n", urg->dist_max );
	printf( "%5d # STEP RESOLUTION\n", urg->step_resolution );
	printf( "%5d # MINIMUM STEP\n", urg->step_min );
	printf( "%5d # MAXIMUM STEP\n", urg->step_max );
	printf( "%5d # STEP IN FRONT OF URG\n", urg->step_front );
	printf( "%5d # REVOLUTION [rpm]\n", urg->revolution );
}
