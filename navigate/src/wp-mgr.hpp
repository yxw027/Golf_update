#ifndef WP_MGR_HPP
#define WP_MGR_HPP

#include <stdio.h>
#include <stdlib.h>
#include "config.hpp"
#include "localizer.hpp"
#include "wp.hpp"

// *** 基本クラス。WPファイルから読み込んで順次出力 *************************
class WpMgr_Base
{
protected:
	unsigned int wp_num;		// 登録したWPの数
	unsigned int wp_cnt;		// WPのカウント（基本は1がスタート）
	wp_gl *wp;					// WP

	config_property conf;
	enum ROBOT_STATUS robot_status;	// ロボットの状態
	
	bool flag_update_wp;		// WPを更新 (true:更新, false:更新しない)
	double offset_gl[ 2 ];		// WP乗り換え開始の距離
	
	void setStartWPid( unsigned int i );
	void calcWPOffset( void );
	
public:
	WpMgr_Base( void ) : wp_cnt( 1 ), wp_num( 1 ) { }
	~WpMgr_Base( void ){ free( wp ); }

	void printAllWP( void );
	void printWPcurrent( void );
	bool chkGoal( void );

	
	ROBOT_STATUS getRobotStatus( void ){ return robot_status; }
	void setRobotStatus( ROBOT_STATUS s ){ robot_status = s; }
	void setFlagUpdateWP( bool u ){ flag_update_wp = u; }
	unsigned int getOrderCuttingUnit( int i ){ return wp[ wp_cnt + i ].flag_cut; }
	double getVel( int i ){ return wp[ wp_cnt + i ].v; }
	
	wp_gl getWPcurrent( void );	// WPファイルから読み込んだWPを順次出力

	void CountUpWP( void ){ wp_cnt++; }
	bool chkUpdateWP( void ){ return flag_update_wp; }

	virtual bool chkOverLine( localizer *odm );
	virtual void initilize( const config_property c, unsigned int id, ROBOT_STATUS status );
	virtual void printWP( wp_gl *w );
	virtual wp_gl getWP( localizer *odm );		// WP出力用関数（クラス毎に変更）
};
// *** ポジショニング用WPを追加 *************************
class WpMgr_Positioning : public WpMgr_Base
{
protected:
	wp_gl wp_positioning[ 2 ];
	
	bool flag_positioning_forward;	// ture:前進, false:後退
	double length4positioning;		// 調整距離[m]を設定
	double velocity4positioning;	// ポジショニング中の速度設定[m/s]
	unsigned int positioning_loop_num;
public:
	WpMgr_Positioning( void ) { }
	~WpMgr_Positioning( void ){ }

	void calcWP4Positioning( void );
	wp_gl getWP4Positioning( localizer *odm );
	
	virtual bool chkOverLine( localizer *odm );
	virtual void initilize( const config_property c, unsigned int id, ROBOT_STATUS status );
	virtual void printWP( wp_gl *w );
	virtual wp_gl getWP( localizer *odm );		// WP出力用関数（クラス毎に変更）
};
// *** 先読みWPを追加 *************************
class WpMgr_Proactive : public WpMgr_Positioning
{
protected:
	unsigned int proactive_wp_cnt;	// 先読みWPのカウント

public:
	WpMgr_Proactive( void ) { }
	~WpMgr_Proactive( void ){ }

	wp_gl getWP4Proactive( localizer *odm );	// 1秒先のWPを出力
	
	virtual bool chkOverLine( localizer *odm );
	virtual void initilize( const config_property c, unsigned int id, ROBOT_STATUS status );
	virtual void printWP( wp_gl *w );
	virtual wp_gl getWP( localizer *odm );		// WP出力用関数（クラス毎に変更）
};
#endif
