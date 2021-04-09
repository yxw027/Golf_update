 /*
  * Data : 2021.04.06
  * Author : T.Hasegawa
  */
#include <stdlib.h>
#include <string.h>
//#include <ssmtype/spur-odometry.h>
//#include "GraphDrawerWP.hpp"
#include <math.h>
#include "createWP.hpp"
#include "utility.hpp"

void createWP_Base::initialize( char *path )
{
	//char filename[ STRLEN ];
	//sprintf( filename, "%s/WP_raw.dat", path );
	//temp_fp = isValidFile( filename, "w", "dat" );
	
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

//static unsigned long counter = 1;
static bool flag_first_loop = true;
static bool flag_second_loop = true;
static double pos_prev[ 3 ] = { 0 };
//static double init_pose[ 3 ] = { 0 };
//static double vel_d = 0.0; // [m/s]
//static unsigned int cflag = 0;
//static unsigned int atype = 0;
//static unsigned int gid = 0;
static bool flag_vehicleDirection_old;
static double WP_theta_old;
bool createWP_Base::setPose( localizer *data )
{
	bool ret = false;
	if( data->status ){
		
		if( flag_first_loop ){
		
			fprintf( temp_fp, "%6d %15.4f %15.4f %15.4f	%15d %25d %12d\n", wp.id, data->estPose.x, data->estPose.y, wp.v, wp.flag_cut, wp.area_type, wp.gain_id );

			wp_size++;
			
//			wp.id++;
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
//			fprintf(stderr,"dL=%lf, th=%lf\n",dL, dTheta*180/M_PI );

			if( ( dL > distThre ) || ( fabs( dtheta ) > angThre ) || ( flag_vehicleDirection_old != data->dir ) ){
				
				if( flag_vehicleDirection_old == data->dir ){ // data->flag_slip(true:前進, false:後退)
					if( data->dir ){
						wp.v = data->gnss_vel[ _V ];
					} else {
						wp.v = -1.0 * data->gnss_vel[ _V ];
					}
				} else { // 前後退の切替え時
					double WP_theta = atan2( data->estPose.y - pos_prev[ _Y ], data->estPose.x - pos_prev[ _X ] );
//					printf("%f - %f = %f\n", WP_theta, WP_theta_old, fabs( WP_theta - WP_theta_old ) );	// 確認用
					if( fabs( WP_theta - WP_theta_old ) < M_PI/4.0 ){
						if( flag_vehicleDirection_old ){
							wp.v = data->gnss_vel[ _V ];
						} else {
							wp.v = -1.0 * data->gnss_vel[ _V ];
						}
					} else {
						if( data->dir ){
							wp.v = data->gnss_vel[ _V ];
						} else {
							wp.v = -1.0 * data->gnss_vel[ _V ];
						}
					}
				}
				wp.v = ( ( double )( int )( wp.v * 10 ) ) / 10.0;	// 小数点以下１桁にする
				wp.id++;
				fprintf( temp_fp, "%6d %15.4f %15.4f %15.4f	%15d %25d %12d\n", wp.id, data->estPose.x, data->estPose.y, wp.v, wp.flag_cut, wp.area_type, wp.gain_id );
				wp_size++;
				
				WP_theta_old = atan2( data->estPose.y - pos_prev[ _Y ], data->estPose.x - pos_prev[ _X ] );
				pos_prev[ _X ] = data->estPose.x;
				pos_prev[ _Y ] = data->estPose.y;
				pos_prev[ _YAW ] = data->estPose.theta;
				// WPファイル確認用
//				fprintf( fp4chk, "%f %f %f %f %f\n", time, data->estPose.x, data->estPose.y, WP_theta_old, vel_d );
								
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
	fprintf( savefp, "#	Start Positionn" );
	fprintf( savefp, "# %15.4f %15.4f %15.4f\n",  initPose[ _X ], initPose[ _Y ], initPose[ _YAW ] );
//	fclose( savefp );
}
void createWP_Base::saveWPFile( void )
{
	fclose( temp_fp );
	openWPfile( "WP_tmp.dat", path, "w" );
	openWPfile( "WP_raw.dat", path, "r" );
	
	writeWPsize( );
	// ...
	for( int i = 0 ; i < wp_size ; i++ ){
		wp.id = atoi( temp_fp );
		wp.x = atof( temp_fp );
		wp.y = atof( temp_fp );
		wp.v = atof( temp_fp );
		wp.flag_cut = atoi( temp_fp );
		wp.area_type = atoi( temp_fp );
		wp.gain_id = atoi( temp_fp );
		fprintf( savefp, "%6d %15.4f %15.4f %15.4f	%15d %25d %12d\n", wp.id, wp.x, wp.y, wp.v, wp.flag_cut, wp.area_type, wp.gain_id );
	}
	writeInitPose( );
	
	writeWPsize( );
}

void createWP_Base::writeWPsize( void )
{
	fprintf( savefp, "#    Num. of WP\n" );
	fprintf( savefp, "    %d\n", wp_size );
	fprintf( savefp, "#    ID    WP_x[m]         WP_y[m]          Velocity[m/s]		CUT_Flag[1:down:2:up]		area_type	gain_id\n" );
}
