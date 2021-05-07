/*
 * Date : 2019.02.01
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <ssmtype/spur-odometry.h>
#include "utility.hpp"
#include "GraphDrawerURG.hpp"

void GraphDrawerURG::setScan( urg_fs *urg, double t )
{
	scan = *urg;
	if( flag_save ){
		savefile.log2txt( urg, t );
	}
}
void GraphDrawerURG::drawGraph( void )
{
	fprintf( gp, "p " );

	if( flag_laser )
		fprintf( gp, " '-' w l lc rgb 'green', " );

	if( flag_intensity )
		fprintf( gp, "'-' pt 7 ps 0.2 pal" );
	else
		fprintf( gp, "'-' pt 7 ps 0.2 lc rgb 'red'" );
				
	fprintf( gp, "\n" );

	if( flag_laser ){
		for( int i = 0 ; i < scan.size ; i++ ){
			fprintf( gp, "0 0\n%lf %lf\n\n", scan.length[ i ]*cos( scan.angle[ i ] ), scan.length[ i ]*sin( scan.angle[ i ] ) );
		}
		fprintf( gp, "e\n" );
	}
	for( int i = 0 ; i < scan.size ; i++ ){
		fprintf( gp, "%lf %lf %d\n", scan.length[ i ]*cos( scan.angle[ i ] ), scan.length[ i ]*sin( scan.angle[ i ] ), scan.intensity[ i ] );
	}
	fprintf( gp, "e\n" );
	fflush( gp );
}
void GraphDrawerURG::printProperty( urg_property *urg )
{
	printf( "\n<URG INFO>\n" );
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
