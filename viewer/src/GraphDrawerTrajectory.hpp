 /*
  * Date : 2018.07.09
  * Update : 2019.09.05
  * Author : T.Hasegawa
  */
#include <stdio.h>
#include <cstddef>
#include <ssmtype/spur-odometry.h>
#include "localizer.hpp"
#include "config.hpp"
#include "wp.hpp"
#include "GraphDrawer.hpp"
#include "log2txt.hpp"
#include "wp-mgr.hpp"

#define WP_DATA_MAX 		100//500
#define PLOT_DATA_MAX		WP_DATA_MAX * 4

using namespace std;

// ********************* SSMからWPを受け取り表示 ***********************
class GraphDrawerTrajectory : public GraphDrawer_RobotBase
{
protected:
	Log2Txt_Control savefile_control;
	Log2Txt_Localizer savefile_localizer;
	bool flag_save_control;
	bool flag_save_localizer;

	config_property conf;
	// 推定ポーズの蓄積
	double pos_data_ring[ PLOT_DATA_MAX ][ 2 ];
	int pos_data_ring_head;
	int pos_data_ring_cnt;
	// WPの蓄積
	double wp_data_ring[ WP_DATA_MAX ][ 2 ];
	int wp_id_ring[ WP_DATA_MAX ];
	int wp_data_ring_head;
	int wp_data_ring_cnt;
	// ロボットの描写パラメータ
	double robot_pos[ 3 ];
	//double robot[ 5 ][ 2 ];
	
	//void fprintf_robot( double *edges, int edge_num, double *pos );

public:
	GraphDrawerTrajectory( void ) : pos_data_ring_head( 0 ), pos_data_ring_cnt( 0 ), wp_data_ring_head( 0 ), wp_data_ring_cnt( 0 ), flag_save_control( false ), flag_save_localizer( false ) {  }
	~GraphDrawerTrajectory( void )
	{
		if( flag_save_localizer ){
			savefile_localizer.closeSaveFile( );
		}
		if( flag_save_control ){
			savefile_control.closeSaveFile( );
		}
	}
	
	virtual void setParameters( config_property *c, bool sflag, bool cflag );
	virtual void setWP( wp_gl *wp );
	virtual void setPose( localizer *est, double t );
	void setControlInfo( control *data, double t );
	
	virtual void drawGraph( void );
};
// ********************* SSMから先読みWPを受け取り表示 && 本来のWPも同時に表示 ***********************
class GraphDrawerTrajectory_Proactive : public GraphDrawerTrajectory
{
protected:
	// 先読みWPの蓄積
	double wp_proactive_data_ring[ WP_DATA_MAX ][ 2 ];
//	int wp_proactive_id_ring[ WP_DATA_MAX ];
	int wp_proactive_data_ring_head;
	int wp_proactive_data_ring_cnt;

	WpMgr_Base wp_mgr;

public:
	GraphDrawerTrajectory_Proactive( void ) : wp_proactive_data_ring_head( 0 ), wp_proactive_data_ring_cnt( 0 ) {  }
	~GraphDrawerTrajectory_Proactive( void ) { }

	virtual void setParameters( config_property *c, bool sflag, bool cflag );
	virtual void setWP( wp_gl *wp );
	virtual void setPose( localizer *est, double t );
	virtual void drawGraph( void );
};

