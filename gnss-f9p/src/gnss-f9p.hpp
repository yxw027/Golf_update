#ifndef _GNSS_F9P_HPP_
#define _GNSS_F9P_HPP_

#define SNAME_GNSS_F9P "RTK-GNSS-F9P"

#define UNKNOWN		0
#define SINGLE_FIX	1
#define DGPS_FIX	2
#define RTK_FIX		4
#define RTK_FLOAT	5

typedef struct
{
	double x;
	double y;
	double z;
} my_position_t;

typedef struct
{
	bool status;
	double utc_time;
	
	double latitude;	// 緯度(deg)
	double longitude;	// 経度(deg)
	double height;		// 楕円対高(m)
	int posStatus;		// FIX or FLOAT, etc
	double elevation;	// 標高(m)
	double Geoid_height;	// ジオイド高(m)
	my_position_t enu;
	my_position_t ecef;
	
} rtk_gnss_f9p;

#endif // _GNSS_F9P_HPP_
