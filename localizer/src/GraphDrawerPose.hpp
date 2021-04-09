 /*
  * Data : 2019.05.30
  * Update : 2019.09.02
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_POSE_HPP_
#define _GRAPH_DRAWER_POSE_HPP_

#include "GraphDrawer.hpp"
#include "config.hpp"
#include "log2txt.hpp"
#include "localizer.hpp"

#define PLOT_DATA_MAX		500 * 4

class GraphDrawerPose : public GraphDrawer_RobotBase
{
private:
	bool flag_save;
	Log2Txt_Localizer savefile;
	
	// 推定ポーズの蓄積（localizer）
	double pos_data_ring[ PLOT_DATA_MAX ][ 2 ];
	int pos_data_ring_head;
	int pos_data_ring_cnt;
	// ロボットの描写パラメータ
	double robot_pos[ 3 ];

public:
	GraphDrawerPose( void ) : pos_data_ring_head( 0 ), pos_data_ring_cnt( 0 ) { }
	~GraphDrawerPose( void )
	{
		if( flag_save ) savefile.closeSaveFile( );
	}

	void setParameters( config_property *c, bool flag );
	void setPose( localizer *data, double t );

	virtual void drawGraph( void );
};

#endif


