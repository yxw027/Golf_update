 /*
  * Data : 2021.04.06
  * Author : T.Hasegawa
  */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "createWP.hpp"
#include "utility.hpp"

void createWP_Base::initialize( char *p )
{
	sprintf( path, "%s", p );
	openWPfile( "WP_raw.dat", path, "w" );
	
	wp.id = 1;
	wp.x = wp.y = wp.theta = 0.0;
	wp.v = 0.0;
	wp.flag_cut = 0;
	wp.area_type = 0;
	wp.gain_id = 0;
	
	for( int i = 0 ; i < 3 ; i++ ){
		initPose[ i ] = 0.0;
	}
}
void createWP_Base::openWPfile( const char *file, char *path, const char* mode )
{
	char filename[ STRLEN ];
	char flag_mode[ STRLEN ];
	sprintf( flag_mode, "%s", mode );
	
	if( !strncmp( "WP_raw.dat", file, 10 ) ){
		sprintf( filename, "%s/%s", path, file );
		temp_fp = isValidFile( filename, flag_mode, "dat" );	// 要確認
	} else if( !strncmp( "WP_tmp.dat", file, 10 ) ){
		sprintf( filename, "%s/%s", path, file );
		savefp = isValidFile( filename, flag_mode, "dat" );	// 要確認
	} else {
		fprintf( stderr, "Error. Save file name might be wrong.\n" );
		exit( EXIT_FAILURE );
	}
}

static bool flag_first_loop = true;
static bool flag_second_loop = true;
static double pos_prev[ 3 ] = { 0 };
static bool flag_vehicleDirection_old = true;
static double WP_theta_old;
bool createWP_Base::setPose( localizer *data )
{
	bool ret = false;
	if( data->status ){
		
		if( flag_first_loop ){
			wp.x = data->estPose.x;
			wp.y = data->estPose.y;
			fprintf( temp_fp, "%6d %15.4f %15.4f %15.4f	%15d %25d %12d\n", wp.id, wp.x, wp.y, wp.v, wp.flag_cut, wp.area_type, wp.gain_id );

			wp_size++;
			
			pos_prev[ _X ] = data->estPose.x;
			pos_prev[ _Y ] = data->estPose.y;
			pos_prev[ _YAW ] = data->estPose.theta;
			flag_first_loop = false;
			flag_vehicleDirection_old = data->dir;
			WP_theta_old = data->estPose.theta;

			initPose[ _X ] = data->estPose.x;
			initPose[ _Y ] = data->estPose.y;

			return true;
		
		} else {
		
			double dx = data->estPose.x - pos_prev[ _X ];
			double dy = data->estPose.y - pos_prev[ _Y ];
			double dL = sqrt( dx * dx + dy * dy );
			double dtheta = trans_q( data->estPose.theta - pos_prev[ _YAW ] );
//			fprintf(stderr,"dL=%lf, th=%lf\n",dL, dtheta*180/M_PI );

			if( ( dL > distThre ) || ( fabs( dtheta ) > angThre ) || ( flag_vehicleDirection_old != data->dir ) ){
				
				if( flag_vehicleDirection_old == data->dir ){ // data->dir(true:前進, false:後退)
					if( data->dir ){
						wp.v = data->estPose.v;
					} else {
						wp.v = -1.0 * data->estPose.v;
					}
				} else { // 前後退の切替え時
					double WP_theta = atan2( data->estPose.y - pos_prev[ _Y ], data->estPose.x - pos_prev[ _X ] );
//					printf("%f - %f = %f\n", WP_theta, WP_theta_old, fabs( WP_theta - WP_theta_old ) );	// 確認用
					if( fabs( WP_theta - WP_theta_old ) < M_PI/4.0 ){
						if( flag_vehicleDirection_old ){
							wp.v = data->estPose.v;
						} else {
							wp.v = -1.0 * data->estPose.v;
						}
					} else {
						if( data->dir ){
							wp.v = data->estPose.v;
						} else {
							wp.v = -1.0 * data->estPose.v;
						}
					}
				}
				wp.v = ( ( double )( int )( wp.v * 100 ) ) / 100.0;	// 小数点以下2桁にする
				wp.id++;
				wp.x = data->estPose.x;
				wp.y = data->estPose.y;
				fprintf( temp_fp, "%6d %15.4f %15.4f %15.4f	%15d %25d %12d\n", wp.id, wp.x, wp.y, wp.v, wp.flag_cut, wp.area_type, wp.gain_id );
				wp_size++;
				
				WP_theta_old = atan2( data->estPose.y - pos_prev[ _Y ], data->estPose.x - pos_prev[ _X ] );
				pos_prev[ _X ] = data->estPose.x;
				pos_prev[ _Y ] = data->estPose.y;
				pos_prev[ _YAW ] = data->estPose.theta;
								
				if( flag_second_loop ){
					flag_second_loop = false;
					initPose[ _YAW ] = atan2( dy, dx );
				}
				ret = true;
			}
			flag_vehicleDirection_old = data->dir;
		}
	}
	return ret;
}

void createWP_Base::writeInitPose( void )
{
	fprintf( savefp, "#	Start Position\n" );
	fprintf( savefp, "# %15.4f %15.4f %15.4f\n",  initPose[ _X ], initPose[ _Y ], initPose[ _YAW ] );
}
void createWP_Base::saveWPFile( void )
{
	fclose( temp_fp );
	openWPfile( "WP_tmp.dat", path, "w" );
	openWPfile( "WP_raw.dat", path, "r" );
	
	writeWPsize( );

	for( int i = 0 ; i < wp_size ; i++ ){
		wp.id = atoi( getWord( temp_fp ) );
		wp.x = atof( getWord( temp_fp ) );
		wp.y = atof( getWord( temp_fp ) );
		wp.v = atof( getWord( temp_fp ) );
		wp.flag_cut = atoi( getWord( temp_fp ) );
		wp.area_type = atoi( getWord( temp_fp ) );
		wp.gain_id = atoi( getWord( temp_fp ) );
		fprintf( savefp, "%6d %15.4f %15.4f %15.4f	%15d %25d %12d\n", wp.id, wp.x, wp.y, wp.v, wp.flag_cut, wp.area_type, wp.gain_id );
	}
	writeInitPose( );

}

void createWP_Base::writeWPsize( void )
{
	fprintf( savefp, "#    Num. of WP\n" );
	fprintf( savefp, "    %d\n", wp_size );
	fprintf( savefp, "#    ID    WP_x[m]         WP_y[m]          Velocity[m/s]		CUT_Flag[1:down:2:up]		area_type	gain_id\n" );
}
