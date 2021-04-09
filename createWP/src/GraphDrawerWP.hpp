 /*
  * Data : 2019.05.30
  * Update : 2019.09.02
  * Update : 2019.09.04
  * Author : T.Hasegawa
  */
#ifndef _GRAPH_DRAWER_WP_HPP_
#define _GRAPH_DRAWER_WP_HPP_

#include "GraphDrawer.hpp"
#include "config.hpp"
#include "localizer.hpp"
#include "createWP.hpp"

#define PLOT_DATA_MAX		1000//500 * 4

class GraphDrawerWP : public GraphDrawer_RobotBase
{
private:
	createWP_Base create_wp;
	
//	config_property conf;
	// 推定ポーズの蓄積（localizer）
	double pos_data_ring[ PLOT_DATA_MAX ][ 2 ];
	int pos_data_ring_head;
	int pos_data_ring_cnt;
	// WPの蓄積
	double wp_data_ring[ PLOT_DATA_MAX ][ 2 ];
	int wp_data_ring_head;
	int wp_data_ring_cnt;
	// ロボットの描写パラメータ
	double robot_pos[ 3 ];
//	double robot[ 5 ][ 2 ];

public:
	GraphDrawerWP( void ) : pos_data_ring_head( 0 ), pos_data_ring_cnt( 0 ), wp_data_ring_head( 0 ), wp_data_ring_cnt( 0 ) { }
	~GraphDrawerWP( void ) { }

	void setParameter( config_property *c, char *path );
	void setPose( localizer *data );
//	void initWPFile( void );
	void writeSaveFile( void );
//	virtual void saveWPFile( localizer *data, double time );

	void drawGraph( void );
};

#endif


