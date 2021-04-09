#ifndef _GRAPH_DRAWER_WP_HPP_
#define _GRAPH_DRAWER_WP_HPP_
 /*
  * Date : 2019.09.30
  * Author : T.Hasegawa
  */
#include <stdio.h>
#include <cstddef>
#include "wp.hpp"
#include "wp-mgr.hpp"
#include "GraphDrawer.hpp"
#include "config.hpp"

using namespace std;

// *** WP-viewer用WP Managerクラスを追加 *************************
class WpMgr_Veiewer : public WpMgr_Base
{
public:
	WpMgr_Veiewer( void ) { }
	~WpMgr_Veiewer( void ){ }
	
	unsigned int getWPid( unsigned int i ){ return wp[ i ].id; }
	double getWPx( unsigned int i ){ return wp[ i ].x; }
	double getWPy( unsigned int i ){ return wp[ i ].y; }
	double getWPth( unsigned int i ){ return wp[ i ].theta; }
	double getWPvel( unsigned int i ){ return wp[ i ].v; }
	unsigned int getNumWP( void ){ return wp_num; }
	
	virtual bool chkOverLine( localizer *odm ){ WpMgr_Base::chkOverLine( odm ); }
	virtual void initilize( const config_property c, unsigned int id, ROBOT_STATUS status ) { WpMgr_Base::initilize( c, id, status ); }
	virtual void printWP( wp_gl *w ){ WpMgr_Base::printWP( w ); }
	virtual wp_gl getWP( localizer *odm ){ WpMgr_Base::getWP( odm ); }	// WP出力用関数（クラス毎に変更）
};

class GraphDrawerWP : public GraphDrawer
{
private:
	WpMgr_Veiewer wp_mgr;
	unsigned int wp_size;	// 現在のWPの数
	bool flag_red4backward; // バック走行の確認
	
	config_property conf;
	bool flag_wpfile_config;
	void checkWP( void );

public:
	GraphDrawerWP( void ) : wp_size( 0 ), flag_red4backward( false ) { }
	~GraphDrawerWP( void ) { }
	
	void setParameters( const config_property c, bool f, const char *file );
	void updateWP( void );

	virtual void drawGraph( void );
};
#endif
