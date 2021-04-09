/*
 * Author : T.Hasegawa
 */
#include <stdlib.h>
#include <math.h>

#include "GraphDrawerPCD.hpp"

using namespace std;

// PCL Visualizer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
// Point Cloud Color Hndler
pcl::visualization::PointCloudColorHandler< PointType >::Ptr handler;
// Point Cloud
pcl::PointCloud< PointType >::Ptr cloud ( new pcl::PointCloud< PointType > );

void GraphDrawerPCD::initialize( bool laser, bool intensity )
{
	flag_intensity = intensity;
	flag_laser = laser;
		
	viewer->addCoordinateSystem( 3.0, "coordinate" );
	viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
	viewer->initCameraParameters( );
	viewer->setCameraPosition( -20.0, 0.0, 20.0, 0.5, 0.0, 0.8, 0 );	// Top

//***** Point Cloud Color Hndler
	const std::type_info& type = typeid( PointType );
	if( type == typeid( pcl::PointXYZ ) ){
		std::vector<double> color;
		color[ 0 ] = color[ 1 ] = color[ 2 ] = 255.0;
		boost::shared_ptr< pcl::visualization::PointCloudColorHandlerCustom< PointType > > color_handler( new pcl::visualization::PointCloudColorHandlerCustom< PointType >( color[ 0 ], color[ 1 ], color[ 2 ] ) );
		handler = color_handler;
	} else if( type == typeid( pcl::PointXYZI ) ){
		boost::shared_ptr< pcl::visualization::PointCloudColorHandlerGenericField< PointType > > color_handler( new pcl::visualization::PointCloudColorHandlerGenericField< PointType >( "intensity" ) );
		handler = color_handler;
	} else if( type == typeid( pcl::PointXYZRGBA ) ){
		boost::shared_ptr< pcl::visualization::PointCloudColorHandlerRGBField< PointType > > color_handler( new pcl::visualization::PointCloudColorHandlerRGBField< PointType >( ) );
		handler = color_handler;
	} else{
		throw std::runtime_error( "This PointType is unsupported." );
	}
}
void GraphDrawerPCD::setScan( urg_fs *urg )
{
	scan = *urg;
	
	//点群の大きさを指定
	cloud->width = urg->size;
	cloud->height = 1;
	cloud->points.resize( cloud->width * cloud->height );

	//点群の代入
	//	h:110cm, d:400cm (w:385)
	double height = 1.1;
	double theta = atan( 1.1 / 3.85 );
	for( int i = 0 ; i < cloud->points.size( ) ; i++ ){
		// 座標変換
		double xx = scan.length[ i ] * cos( scan.angle[ i ] );
		double yy = scan.length[ i ] * sin( scan.angle[ i ] );
		double tx = xx * cos( theta );
		double ty = yy;
		double tz = -1.0 * xx * sin( theta ) + height;
		cloud->points[ i ].x = tx;
		cloud->points[ i ].y = ty;
		cloud->points[ i ].z = tz;
		cloud->points[ i ].intensity = scan.intensity[ i ];
	}

}
bool GraphDrawerPCD::drawGraph( void )
{
	if( viewer->wasStopped( ) ) return false;
	
	// Update Viewer
	viewer->spinOnce( );

	// Update Point Cloud
	if( cloud ){
		handler->setInputCloud( cloud );
		if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
			viewer->addPointCloud( cloud, *handler, "cloud" );
		}
	}
	return true;
}
#include <pcl/io/pcd_io.h>
static unsigned long cnt = 0;
void GraphDrawerPCD::savePCD( void )
{
	char filename[ 50 ];
	sprintf( filename, "../pcd/pcd%ld.pcd", cnt );
	pcl::io::savePCDFileASCII ( filename , *cloud );
	cnt++;

}

