#ifndef OBP_HPP
#define OBP_HPP

#include "config.hpp"
#include "detectObstacle.hpp"
#include "urg.hpp"

// **************************************
// ******* 障害物検知の基本クラス ********
// *************************************
class detectObstacle_Base
{
protected:
	int skip_point;		// URGデータのダウンサイジング数
	int pointNThre;		// 障害物と認識するポイント数
	obp_fs obp;
	int area_type;
//	double vel_max, angvel_max;
	config_property conf;
	
	Area dec[ NUM_AREA_TYPE ];
	Area stop[ NUM_AREA_TYPE ];
	Area avoid[ NUM_AREA_TYPE ];

	void setSkipPoint( int a ){ skip_point = a; }
	void setPointNThre( int a ){ pointNThre = a; }
	void printAreaData( void );
	void printObstData( void );
	
public:
	detectObstacle_Base( void ) : 	skip_point( 1 ), pointNThre( 5 ) { }
	~detectObstacle_Base( void ) {  }
 
	void setAreaType( int a ){ area_type = a; }
//	void setVel( double v ){ vel_max = v; }

	obp_fs getObpData( void ){ return obp; }
	
	virtual void setParameter( const config_property cnf );
	virtual void chkObstacle( urg_fs *urg ) = 0;
};
// *******************************************
// ******* 2D平面での障害物検知クラス ********
// *******************************************
class detectObstacle_2D : public detectObstacle_Base
{
protected:

public:
	detectObstacle_2D( void ) { }
	~detectObstacle_2D( void ) {  }

	virtual void setParameter( const config_property cnf ){ detectObstacle_Base::setParameter( cnf ); }
	virtual void chkObstacle( urg_fs *urg );
};
// ***************************************************
// ******* 2D PCDを3D変換後の障害物検知クラス ********
// ***************************************************
class detectObstacle_3D : public detectObstacle_Base
{
private:
	double height;
	double theta;

public:
	detectObstacle_3D( void ) { }
	~detectObstacle_3D( void ) { }

	virtual void setParameter( const config_property cnf );
	virtual void chkObstacle( urg_fs *urg );
};

#endif // OBP_HPP
