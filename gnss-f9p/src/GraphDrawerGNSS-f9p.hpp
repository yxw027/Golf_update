 /*
  * GraphDrawerGNSS-f9p.hpp
  * Data : 2019.05.30
  * Update : 2019.08.01
  * Update : 2019.08.17
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_GNSS_F9P_HPP_
#define _GRAPH_DRAWER_GNSS_F9P_HPP_

#include <vector>
#include "GraphDrawer.hpp"
#include "gnss-f9p.hpp"
#include "rtknavi-f9p.hpp"
#include "log2txt.hpp"

#define DATA_MAX		1000

class GraphDrawerGNSS : public GraphDrawer_RobotBase
{
private:
	bool flag_save;
	Log2Txt_GNSS_F9P savefile;

	// 推定ポーズの蓄積（RTK Fix）
	double rtk_fix_data_ring[ DATA_MAX ][ 2 ];
	int rtk_fix_data_ring_head;
	int rtk_fix_data_ring_cnt;
	// 推定ポーズの蓄積（RTK Float）
	double rtk_float_data_ring[ DATA_MAX ][ 2 ];
	int rtk_float_data_ring_head;
	int rtk_float_data_ring_cnt;
	// 推定ポーズの蓄積（DGPS Fix）
	double dgps_fix_data_ring[ DATA_MAX ][ 2 ];
	int dgps_fix_data_ring_head;
	int dgps_fix_data_ring_cnt;
	// 推定ポーズの蓄積（Single Fix）
	double single_fix_data_ring[ DATA_MAX ][ 2 ];
	int single_fix_data_ring_head;
	int single_fix_data_ring_cnt;
	// 推定ポーズの蓄積（unknown）
	double unknown_data_ring[ DATA_MAX ][ 2 ];
	int unknown_data_ring_head;
	int unknown_data_ring_cnt;
	// ロボットの描写パラメータ
	double robot_pos[ 3 ];
//	double robot[ 5 ][ 2 ];
	
//	void fprintf_robot( double *edges, int edge_num, double *pos );

public:
	GraphDrawerGNSS( void ) : rtk_fix_data_ring_head( 0 ), rtk_fix_data_ring_cnt( 0 ), rtk_float_data_ring_head( 0 ), rtk_float_data_ring_cnt( 0 ),
								dgps_fix_data_ring_head( 0 ), dgps_fix_data_ring_cnt( 0 ), single_fix_data_ring_head( 0 ), single_fix_data_ring_cnt( 0 ),
								unknown_data_ring_head( 0 ), unknown_data_ring_cnt( 0 ) { }
	~GraphDrawerGNSS( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameters( config_property *c, bool flag );
	void setPose( rtk_gnss_f9p *gnss, double t );

	virtual void drawGraph( void );
};

#endif


