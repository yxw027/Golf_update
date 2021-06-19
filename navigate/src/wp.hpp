#ifndef WP_HPP
#define WP_HPP

#define WP_SNAME	"wp_gl"

#include "framework.hpp"

#ifdef Use_Follow_Circle
#define _R	2
#else
#endif

enum ROBOT_STATUS {
	START,
	NAVI,
	GOAL,
	POSITIONING,
	DUMMY
};

typedef struct {
	unsigned int id;
	double x, y, theta;
	double v;
	unsigned int flag_cut;
	int area_type;
	int gain_id;
	
#ifdef Use_Follow_Circle
	double circle[ 3 ];	//	[0]center(x), [1]center(y), [2]radius
#else
#endif

} wp_gl;


#endif
