/*
 * Date : 2018.07.09
 * Update : 2019.01.12
 * Update : 2019.02.01
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include "GraphDrawerIMU.hpp"
#include "utility.hpp"
#include "imu.hpp"
#include <math.h>

using namespace std;

////////// Gnuplotによる軌跡の描画 //////////

void GraphDrawerIMU::setParameters( int mode, bool flag )
{
	graph_mode = mode;
	flag_save = flag;
	
	gyro_ring_head = gyro_ring_cnt = 0;
	accel_ring_head = accel_ring_cnt = 0;		
	mag_ring_head = mag_ring_cnt = 0;	
	temperature_ring_head = temperature_ring_cnt = 0;	
	est_ang_ring_head = est_ang_ring_cnt = 0;
	
	for( int i = 0 ; i < DATA_MAX ; i++ ){
		for( int j = 0 ; j < 3 ; j++ ){
			time[ i ] = 0.0;
			gyro_ring[ i ][ j ] = 0.0;
			accel_ring[ i ][ j ] = 0.0;
			mag_ring[ i ][ j ] = 0.0;
			temperature_ring[ i ] = 0.0;
			est_ang_ring[ i ][ j ] = 0.0;
		}
	}
	
	if( flag_save ){
		savefile.openSaveFile( "log_imu.dat" );
	}
}

void GraphDrawerIMU::setIMU( imu_fs *imu, double t )
{
	if( flag_save ){
		savefile.log2txt( imu, t );
	}
	gyro_ring[ gyro_ring_head ][ _PITCH ] = imu->angvel[ _PITCH ];
	gyro_ring[ gyro_ring_head ][ _ROLL ] = imu->angvel[ _ROLL ];
	gyro_ring[ gyro_ring_head ][ _YAW ] = imu->angvel[ _YAW ];
	time[ gyro_ring_head ] = t;
	curTime = t;
	gyro_ring_head++;
	if( gyro_ring_head >= DATA_MAX )
		gyro_ring_head = 0;
	if( gyro_ring_cnt < DATA_MAX )
		gyro_ring_cnt++;

	accel_ring[ accel_ring_head ][ _X ] = imu->accel[ _X ];
	accel_ring[ accel_ring_head ][ _Y ] = imu->accel[ _Y ];
	accel_ring[ accel_ring_head ][ _Z ] = imu->accel[ _Z ];
	accel_ring_head++;
	if( accel_ring_head >= DATA_MAX )
		accel_ring_head = 0;
	if( accel_ring_cnt < DATA_MAX )
		accel_ring_cnt++;

	mag_ring[ mag_ring_head ][ 0 ] = imu->mag[ 0 ];
	mag_ring[ mag_ring_head ][ 1 ] = imu->mag[ 1 ];
	mag_ring[ mag_ring_head ][ 2 ] = imu->mag[ 2 ];
	mag_ring_head++;
	if( mag_ring_head >= DATA_MAX )
		mag_ring_head = 0;
	if( mag_ring_cnt < DATA_MAX )
		mag_ring_cnt++;	
		
	temperature_ring[ temperature_ring_head ] = imu->temperature;
	temperature_ring_head++;
	if( temperature_ring_head >= DATA_MAX )
		temperature_ring_head = 0;
	if( temperature_ring_cnt < DATA_MAX )
		temperature_ring_cnt++;					
		
	est_ang_ring[ est_ang_ring_head ][ _PITCH ] = trans_q( imu->estAng[ _PITCH ] );
	est_ang_ring[ est_ang_ring_head ][ _ROLL ] = trans_q( imu->estAng[ _ROLL ] );
	est_ang_ring[ est_ang_ring_head ][ _YAW ] = trans_q( imu->estAng[ _YAW ] );
	est_ang_ring_head++;
	if( est_ang_ring_head >= DATA_MAX )
		est_ang_ring_head = 0;
	if( est_ang_ring_cnt < DATA_MAX )
		est_ang_ring_cnt++;
}
void GraphDrawerIMU::drawGraph( void )
{
	if( gyro_ring_head > 0 ){
					
		fprintf( gp,"reset\n unset mouse\nset grid\n set key outside below\n" );		

		switch( graph_mode ){
		case 1:
			fprintf( gp, "set title 'Angluar velocity [rad/s]'\n" );
			fprintf( gp, "set yrange[-3.5:3.5]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
			fprintf( gp, "set xtics 5\n" );
			fprintf( gp, "p " );
			fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'Roll'," );
			fprintf( gp, " '-' pt 5 lc rgb 'green' t 'Pitch'," );
			fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Yaw'" );
			fprintf( gp, "\n" );
			for( int i = 0 ; i < gyro_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], gyro_ring[ i ][ _ROLL ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < gyro_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], gyro_ring[ i ][ _PITCH ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < gyro_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], gyro_ring[ i ][ _YAW ] );
			}
			fprintf( gp,"e\n" );
			fflush( gp );
			break;
		case 2:
			fprintf( gp, "set title 'Accelaration [g]'\n" );
			fprintf( gp, "set yrange[-15:15]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
			fprintf( gp, "set xtics 5\n" );
			fprintf( gp, "p " );
			fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'X'," );
			fprintf( gp, " '-' pt 5 lc rgb 'green' t 'Y'," );
			fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Z' " );
			fprintf( gp, "\n" );
			for( int i = 0 ; i < accel_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], accel_ring[ i ][ _X ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < accel_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], accel_ring[ i ][ _Y ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < accel_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], accel_ring[ i ][ _Z ] );
			}
			fprintf( gp,"e\n" );
			fflush( gp );
			break;
		case 3:
			fprintf( gp, "set title 'mag [uT]'\n" );
			fprintf( gp, "set yrange[-3:3]\nset xrange[%lf:%lf]\nset cbrange[0:1000]\n", curTime-TIME_WIDTH, curTime );
			fprintf( gp, "set xtics 1\n" );
			fprintf( gp, "p " );
			fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'X'," );
			fprintf( gp, " '-' pt 5 lc rgb 'green' t 'Y'," );
			fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Z' " );
			fprintf( gp, "\n" );
			for( int i = 0 ; i < mag_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], mag_ring[ i ][ _X ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < mag_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], mag_ring[ i ][ _Y ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < mag_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], mag_ring[ i ][ _Z ] );
			}
			fprintf( gp,"e\n" );
			fflush( gp );
			break;
		case 4:
			fprintf( gp, "set title 'Temperature [degree centigrade]'\n" );
			fprintf( gp, "set yrange[0:80]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
			fprintf( gp, "set xtics 1\n" );
			fprintf( gp, "p " );
			fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'temperature'," );
			fprintf( gp, "\n" );
			for( int i = 0 ; i < temperature_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], temperature_ring[ i ] );
			}
			fprintf( gp,"e\n" );
			fflush( gp );
			break;
		case 5:
			fprintf( gp, "set title 'Estimated Angle [rad]'\n" );
			fprintf( gp, "set yrange[-3:3]\nset xrange[%lf:%lf]\n", curTime-TIME_WIDTH, curTime );
			fprintf( gp, "set xtics 5\n" );
			fprintf( gp, "p " );
			fprintf( gp, " '-' pt 5 lc rgb 'blue' t 'Roll'," );
			fprintf( gp, " '-' pt 5 lc rgb 'green' t 'Pitch'," );
			fprintf( gp, " '-' pt 5 lc rgb 'red' t 'Yaw'" );
			fprintf( gp, "\n" );
						
			for( int i = 0 ; i < est_ang_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], est_ang_ring[ i ][ _ROLL ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < est_ang_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], est_ang_ring[ i ][ _PITCH ] );
			}
			fprintf( gp,"e\n" );
			for( int i = 0 ; i < est_ang_ring_cnt ; i++ ){
				fprintf( gp,"%lf %lf\n", time[ i ], est_ang_ring[ i ][ _YAW ] );
			}
			fprintf( gp,"e\n" );
			fflush( gp );
			break;
		default:
			break;
		}
	}
}
void GraphDrawerIMU::printProperty( imu_property *data )
{
	Gprint( "Calibration\n" );
	printf( "Offset for Angular Velocity\n" );
	printf( "roll= %f, pitch= %f, yaw= %f[rad]\n", data->angvel_offset[ _X ], data->angvel_offset[ _Y ], data->angvel_offset[ _X ] );
	printf( "Initial angles and Acceleration of Gravity\n" );
	printf( "roll= %f, pitch= %f[rad]\n", data->initAng[ _ROLL ], data->initAng[ _PITCH ] );
	printf( "Gravity = %f[m/s^2]\n", data->gravity );
}
